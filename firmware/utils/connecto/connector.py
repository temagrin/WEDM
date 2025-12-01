import serial
import struct
from color_CLI import *

DEFAULT_USB_CDC_BAUD_RATE = 921600
SYNC_MARKER = 0xABCD

COMMAND_STRUCT_FORMAT = '<BBIIIIII'
PACKET_HEADER_STRUCT_FORMAT = '<HHB'
STATUS_STRUCT_FORMAT = '<HHHhiiHHHH'

COMMAND_STRUCT_SIZE = struct.calcsize(COMMAND_STRUCT_FORMAT)
PACKET_HEADER_STRUCT_SIZE = struct.calcsize(PACKET_HEADER_STRUCT_FORMAT)
STATUS_STRUCT_SIZE = struct.calcsize(STATUS_STRUCT_FORMAT)


def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


class Packet:
    def __init__(self):
        self._seq_id = 0
        self.commands = []
        self.packet_flags = 0

    def get_seq_id(self):
        return self._seq_id

    def add_command(self, command_id=0, flag1=0, flag2=0, flag3=0, flag4=0, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0):
        if len(self.commands) >= 16:
            raise "max allowed 16 commands in packet"
        ctrl_flags = ((flag4 & 0x1) << 3) | ((flag3 & 0x1) << 2) | ((flag2 & 0x1) << 1) | (flag1 & 0x1)
        self.commands.append(struct.pack(COMMAND_STRUCT_FORMAT, (command_id & 0xFF), (ctrl_flags & 0xFF),
                                         param1, param2, param3, param4, param5, param6))

    def new_packet(self, packet_flags):
        self.commands = []
        self.packet_flags = packet_flags & 0xF
        self._seq_id += 1
        if self._seq_id > 10001:
            self._seq_id = 0

    def render_with_crc_to_bytes(self) -> bytes:
        size_reserved = ((self.packet_flags & 0x0F) << 4) | (len(self.commands) & 0x0F)
        data_part = struct.pack(PACKET_HEADER_STRUCT_FORMAT, SYNC_MARKER, self._seq_id, size_reserved)
        for c in self.commands:
            data_part += c
        crc_value = calculate_crc16(data_part)
        crc_part = struct.pack('<H', crc_value)
        full_packet = data_part + crc_part
        return full_packet


class Status:
    def __init__(self):
        self.buffer_filling = 0
        self.amperage = 0
        self.tension = 0
        self.current_position_x = 0
        self.current_position_y = 0
        self.ack_mask = None
        self.nak_mask = None
        self.seq_id = None
        self.sent_seq_id = False
        self.sent_batch_commands_count = 0
        self.un_confirmation_seq_id = None
        self.packet_confirmation_callback = None
        self._rx_buffer = bytearray()
        self._rx_buffer2 = bytearray()


    def print_status(self):
        print("-=Read status=-")
        print(f"\tbuffer_filling={self.buffer_filling}")
        print(f"\tamperage={self.amperage}")
        print(f"\ttension={self.tension}")
        print(f"\tcurrent_position_x={self.current_position_x}")
        print(f"\tcurrent_position_y={self.current_position_y}")
        print(f"\tack_mask={self.ack_mask}")
        print(f"\tnak_mask={self.nak_mask}")
        print(f"\tseq_id={self.seq_id}")

    def _try_consume_buffer2(self):
        if 6 >= len(self._rx_buffer2):
            return

        if self._rx_buffer2[0] != 188 or self._rx_buffer2[1] != 202:
            return

        text_len = self._rx_buffer2[2]
        packet_end = 5 + text_len

        if packet_end > len(self._rx_buffer2): # дошел еще не весь текст
            return

        payload = self._rx_buffer2[0 : 3+text_len]
        calc_crc = calculate_crc16(payload)
        rec_crc = self._rx_buffer2[packet_end-2] | (self._rx_buffer2[packet_end-1] << 8)

        if calc_crc == rec_crc:
            text = bytes(self._rx_buffer2[3 : 3+text_len]).decode('ascii', errors='ignore')
            del self._rx_buffer2[:packet_end]
            print(f"MK::{YELLOW}{BOLD}{text}{RESET}")

    def _try_consume_buffer(self):
        candidate = self._rx_buffer[:STATUS_STRUCT_SIZE]
        marker, v1, v2, v3, v4, v5, ack_mask, nak_mask, seq_id, r_crc = struct.unpack_from(STATUS_STRUCT_FORMAT,
                                                                                           candidate)
        if marker != SYNC_MARKER:
            return False  # первые 2 байта не маркер - выкидывайте один байт, ждите новый байт

        calc_crc = calculate_crc16(candidate[:-2])
        if calc_crc != r_crc:
            return False  # не прошел проверку crc - выкидывайте один байт, ждите новый байт
        self.buffer_filling = v1
        self.amperage = v2
        self.tension = v3
        self.current_position_x = v4
        self.current_position_y = v5
        self.ack_mask = ack_mask
        self.nak_mask = nak_mask
        self.seq_id = seq_id
        return True  # потребили 24 байта, можно убрать с буфера 24 байта

    def add_incoming_byte(self, incoming):
        # приняли один байт, добавляем в буфер
        self._rx_buffer += incoming
        self._rx_buffer2 += incoming
        self._try_consume_buffer2()
        if len(self._rx_buffer2)>255:
            self._rx_buffer2 = self._rx_buffer2[1:]

        if len(self._rx_buffer) >= STATUS_STRUCT_SIZE:
            if self._try_consume_buffer():
                self._rx_buffer = self._rx_buffer[STATUS_STRUCT_SIZE:]
                self._process_status()
            else:
                self._rx_buffer = self._rx_buffer[-(STATUS_STRUCT_SIZE - 1):]

    def _process_status(self):
        if self.un_confirmation_seq_id and self.seq_id == self.un_confirmation_seq_id:
            # print("-=Обрабатываем результат подтверждения пачки")
            # total_processed = bin(self.ack_mask | self.nak_mask).count('1')
            # if total_processed != self.sent_batch_commands_count:
            #     print("\tОшибка, сумма ack+nak != размеру пачки")
            #     # переотправка пачки
            # nak_bits = [i for i in range(16) if (self.nak_mask & (1 << i))]
            # if nak_bits:
            #     print(f"\tНе приняты команды с индексами {nak_bits}")
            #     # переотправка части команд в пачке
            # ack_bits = [i for i in range(16) if (self.ack_mask & (1 << i))]
            # if ack_bits:
            #     print(f"\tПриняты команды с индексами {ack_bits}")
            #     # отмечаем команды в пачке как обработанные
            self.packet_confirmation_callback(self)
            self.un_confirmation_seq_id = None


class Connector:
    def __init__(self, port_name, connection_timeout=1, read_timeout=1):
        self._port_name = port_name
        self._port = None
        self.status = Status()
        self._read_timeout = read_timeout
        self._connection_timeout = connection_timeout
        self._tx_buffer = None

    def process(self):
        # если чето нужно было отправить - отправляем
        if self._tx_buffer:
            self._port.write(self._tx_buffer)
            self._port.flush()
            self._tx_buffer = None
        # читаем если есть очередной байт в rx буфер статуса
        incoming = self._port.read(1)
        if incoming:
            self.status.add_incoming_byte(incoming)

    def connect(self):
        print("-=Connecting=-")
        self._port = serial.Serial(self._port_name, DEFAULT_USB_CDC_BAUD_RATE, timeout=self._connection_timeout)

    def disconnect(self):
        print("-=Close port=-")
        self._port.close()

    def send_command(self, cmd: Packet, confirmation_callback=None):
        # если uid команды еще не пришел в статусе, значит она еще в работе в мк - новую не шлем
        if self.status.un_confirmation_seq_id is not None:
            return False
        print(f"[{cmd.get_seq_id()}]::{GREEN}{cmd.render_with_crc_to_bytes().hex()}{RESET}")
        self._tx_buffer = cmd.render_with_crc_to_bytes()
        self.status.un_confirmation_seq_id = cmd.get_seq_id()
        self.status.packet_confirmation_callback = confirmation_callback
        self.status.sent_batch_commands_count = len(cmd.commands)
        return True
