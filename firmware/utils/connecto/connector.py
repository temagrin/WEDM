import serial
import struct

DEFAULT_USB_CDC_BAUD_RATE = 921600
SYNC_MARKER = 0xABCD
COMMAND_STRUCT_FORMAT = '<HBIIIIi'
STATUS_STRUCT_FORMAT = '<HHHhQiiiH'

COMMAND_STRUCT_SIZE = struct.calcsize(COMMAND_STRUCT_FORMAT)
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


class Command:
    def __init__(self):
        self._command_id = 0 & 0xF
        self._flag1 = 0 & 0x1
        self._flag2 = 0 & 0x1
        self._flag3 = 0 & 0x1
        self._flag4 = 0 & 0x1
        self._param1 = 0
        self._param2 = 0
        self._param3 = 0
        self._param4 = 0
        self._command_uid = 0

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
        self._command_uid += 1

    def get_command_id(self):
        return self._command_id

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
        self.current_command_uid = None
        self.command_confirmation_callback = None
        self._rx_buffer = bytearray()

    def _parse_buffer(self):
        for i in range(len(self._rx_buffer) - STATUS_STRUCT_SIZE + 1):
            start_marker, = struct.unpack_from('<H', self._rx_buffer, i)
            candidate = self._rx_buffer[i:i + STATUS_STRUCT_SIZE]
            if start_marker == SYNC_MARKER:
                data_without_crc = candidate[:-2]
                calc_crc = calculate_crc16(data_without_crc)
                temp_unpacked = struct.unpack_from(STATUS_STRUCT_FORMAT, candidate)
                if calc_crc == temp_unpacked[8]:
                    self.buffer_filling = temp_unpacked[1]
                    self.amperage = temp_unpacked[2]
                    self.tension = temp_unpacked[3]
                    self.updated_at = temp_unpacked[4]
                    self.current_position_x = temp_unpacked[5]
                    self.current_position_y = temp_unpacked[6]
                    self.last_command_uid = temp_unpacked[7]
                    self._check_new()
                self._rx_buffer = self._rx_buffer[i + STATUS_STRUCT_SIZE:]
                return
        self._rx_buffer = self._rx_buffer[-(STATUS_STRUCT_SIZE - 1):]

    def add_incoming_byte(self, incoming):
        self._rx_buffer += incoming
        if len(self._rx_buffer) >= STATUS_STRUCT_SIZE:
            self._parse_buffer()

    def _check_new(self):
        print("-=Read new status=-")
        print(f"\tupdated_at={self.updated_at}")
        print(f"\tbuffer_filling={self.buffer_filling}")
        print(f"\tamperage={self.amperage}")
        print(f"\ttension={self.tension}")
        print(f"\tcurrent_position_x={self.current_position_x}")
        print(f"\tcurrent_position_y={self.current_position_y}")
        print(f"\tlast_command_uid={self.last_command_uid}")
        if self.last_command_uid == self.current_command_uid and self.command_confirmation_callback:
            self.command_confirmation_callback(self)
            self.current_command_uid = None
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
        if self._tx_buffer:
            self._port.write(self._tx_buffer)
            self._port.flush()
            self._tx_buffer = None
        incoming = self._port.read(1)
        if incoming:
            self.status.add_incoming_byte(incoming)

    def connect(self):
        print("-=Connecting=-")
        self._port = serial.Serial(self._port_name, DEFAULT_USB_CDC_BAUD_RATE, timeout=self._connection_timeout)

    def disconnect(self):
        self._port.close()

    def send_command(self, cmd: Command, confirmation_callback=None):
        if self._tx_buffer is not None and self.status.current_command_uid is None:
            return False
        cmd_bytes = cmd.render_with_crc_to_bytes()
        print("-=Send new command=-")
        print(f"\tcmdId={cmd.get_command_id()}")
        print(f"\tcmdUid={cmd.get_command_uid()}")
        self._tx_buffer = cmd_bytes
        self.status.command_confirmation_callback = confirmation_callback
        self.status.current_command_uid = cmd.get_command_uid()
        return True
