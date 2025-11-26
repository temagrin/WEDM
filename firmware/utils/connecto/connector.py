import serial
import struct
from collections import namedtuple

DEFAULT_USB_CDC_BAUD_RATE = 921600
SYNC_MARKER = 0xABCD

COMMAND_STRUCT_FORMAT = '<HBIIIIi'
STATUS_STRUCT_FORMAT = '<HHHhQiiiH'
COMMAND_STRUCT_SIZE = struct.calcsize(COMMAND_STRUCT_FORMAT)
STATUS_STRUCT_SIZE = struct.calcsize(STATUS_STRUCT_FORMAT)
NamedStatus = namedtuple('NamedStatus', ['marker', 'buffer_filling', 'amperage', 'tension',
                                         'updated_at', 'current_x', 'current_y', 'last_command_uid', 'crc'])


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


class Command:
    def __init__(self):
        self._command_uid = 0
        self._command_id = 0 & 0xF
        self._flag1 = 0 & 0x1
        self._flag2 = 0 & 0x1
        self._flag3 = 0 & 0x1
        self._flag4 = 0 & 0x1
        self._param1 = 0
        self._param2 = 0
        self._param3 = 0
        self._param4 = 0

    def get_command_id(self):
        return self._command_id

    def get_command_uid(self):
        return self._command_uid

    def set_attr(self, command_id=0, flag1=0, flag2=0, flag3=0, flag4=0, param1=0, param2=0, param3=0, param4=0):
        self._command_id = command_id & 0xF
        self._flag1 = flag1 & 0x1
        self._flag2 = flag2 & 0x1
        self._flag3 = flag3 & 0x1
        self._flag4 = flag4 & 0x1
        self._param1 = abs(param1)
        self._param2 = abs(param2)
        self._param3 = abs(param3)
        self._param4 = abs(param4)
        self._command_uid += 1 # каждая новая команда с новым uid


    def _control_to_byte(self) -> int:
        return ((self._command_id & 0xF)
                | ((self._flag1 & 0x1) << 4)
                | ((self._flag2 & 0x1) << 5)
                | ((self._flag3 & 0x1) << 6)
                | ((self._flag4 & 0x1) << 7))

    def render_with_crc_to_bytes(self) -> bytes:
        control_byte_val = self._control_to_byte()
        data_part = struct.pack(COMMAND_STRUCT_FORMAT, SYNC_MARKER, control_byte_val,
                                self._param1, self._param2, self._param3, self._param4, self._command_uid)
        crc_value = calculate_crc16(data_part)
        crc_part = struct.pack('<H', crc_value)
        full_packet = data_part + crc_part
        return full_packet


class Status:
    def __init__(self):
        self.has_new_status = False
        self.updated_at = 0
        self.buffer_filling = 0
        self.amperage = 0
        self.tension = 0
        self.current_position_x = 0
        self.current_position_y = 0
        self.last_command_uid = None
        self.confirmation_command_uid = None
        self.command_confirmation_callback = None
        self._rx_buffer = bytearray()

    def _try_consume_buffer(self):
        start_marker, = struct.unpack_from('<H', self._rx_buffer, 0)
        if start_marker != SYNC_MARKER:
            return False # первые 2 байта не маркер - выкидывайте один байт, ждите новый байт

        data_without_crc = self._rx_buffer[:-2]
        calc_crc = calculate_crc16(data_without_crc)
        unpacked = NamedStatus._make(struct.unpack_from(STATUS_STRUCT_FORMAT, self._rx_buffer[:STATUS_STRUCT_SIZE]))
        if calc_crc != unpacked.crc:
            return False # не прошел проверку crc - выкидывайте один байт, ждите новый байт

        self.buffer_filling = unpacked.buffer_filling
        self.amperage = unpacked.amperage
        self.tension = unpacked.tension
        self.updated_at = unpacked.updated_at
        self.current_position_x = unpacked.current_x
        self.current_position_y = unpacked.current_y
        self.last_command_uid = unpacked.last_command_uid
        self._check_new()
        return True # потребили 30 байт, можно убрать с буфера 30 байт

    def add_incoming_byte(self, incoming):
        # приняли один байт, добавляем в буфер
        self._rx_buffer += incoming
        if len(self._rx_buffer) >= STATUS_STRUCT_SIZE:
            if self._try_consume_buffer():
                self._rx_buffer = self._rx_buffer[STATUS_STRUCT_SIZE:]
            else:
                self._rx_buffer = self._rx_buffer[-(STATUS_STRUCT_SIZE - 1):]

    def _check_new(self):
        print("-=Read new status=-")
        print(f"\tupdated_at={self.updated_at}")
        print(f"\tbuffer_filling={self.buffer_filling}")
        print(f"\tamperage={self.amperage}")
        print(f"\ttension={self.tension}")
        print(f"\tcurrent_position_x={self.current_position_x}")
        print(f"\tcurrent_position_y={self.current_position_y}")
        print(f"\tlast_command_uid={self.last_command_uid}")
        # подтверждаем обработку крайней задачи
        if self.last_command_uid == self.confirmation_command_uid:
            # если нужен колбек у команды - его вызываем
            if self.command_confirmation_callback:
                self.command_confirmation_callback(self)
            # очищаем подтверждение - освобождая место под новую команду
            self.confirmation_command_uid = None
            self.command_confirmation_callback = None


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

    def send_command(self, cmd: Command, confirmation_callback=None):
        # если uid команды еще не пришел в статусе, значит она еще в работе в мк - новую не шлем
        if self.status.confirmation_command_uid is not None:
            return False
        cmd_bytes = cmd.render_with_crc_to_bytes()
        print("-=Send new command=-")
        print(f"\tcmdId={cmd.get_command_id()}")
        print(f"\tcmdUid={cmd.get_command_uid()}")
        self._tx_buffer = cmd_bytes
        self.status.command_confirmation_callback = confirmation_callback
        self.status.confirmation_command_uid = cmd.get_command_uid()
        return True
