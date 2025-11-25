from time import sleep
import serial
import struct


class CommandControlByte:
    def __init__(self, command_id=0, flag1=0, flag2=0, flag3=0, flag4=0):
        self.command_id = command_id & 0xF
        self.flag1 = flag1 & 0x1
        self.flag2 = flag2 & 0x1
        self.flag3 = flag3 & 0x1
        self.flag4 = flag4 & 0x1

    def to_byte(self):
        return ((self.command_id & 0xF)
                | ((self.flag1 & 0x1) << 4)
                | ((self.flag2 & 0x1) << 5)
                | ((self.flag3 & 0x1) << 6)
                | ((self.flag4 & 0x1) << 7))

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

COMMAND_DATA_SIZE = 19
SYNC_MARKER = 0xABCD

def create_command(control: CommandControlByte, param1, param2, param3, param4) -> bytes:
    control_byte_val = control.to_byte()
    data_part = struct.pack('<HBIIII', SYNC_MARKER, control_byte_val, param1, param2, param3, param4)
    if len(data_part) != COMMAND_DATA_SIZE:
        raise ValueError(f"Data part size mismatch: {len(data_part)} != {COMMAND_DATA_SIZE}")
    crc_value = calculate_crc16(data_part)
    crc_part = struct.pack('<H', crc_value)
    full_packet = data_part + crc_part
    return full_packet


STATUS_STRUCT_FORMAT = '<HHHhQiiH'
STATUS_STRUCT_SIZE = struct.calcsize(STATUS_STRUCT_FORMAT)
print(STATUS_STRUCT_SIZE)

def parse_status(buffer: bytes):
    if len(buffer) < STATUS_STRUCT_SIZE:
        return None, buffer

    for i in range(len(buffer) - STATUS_STRUCT_SIZE + 1):
        start_marker, = struct.unpack_from('<H', buffer, i)
        if start_marker == SYNC_MARKER:
            candidate = buffer[i:i+STATUS_STRUCT_SIZE]
            data_without_crc = candidate[:-2]
            received_crc, = struct.unpack_from('<H', candidate, STATUS_STRUCT_SIZE - 2)
            calc_crc = calculate_crc16(data_without_crc)
            if calc_crc == received_crc:
                unpacked = struct.unpack(STATUS_STRUCT_FORMAT, candidate)
                status = {
                    'start_marker': unpacked[0],
                    'bufferFilling': unpacked[1],
                    'amperage': unpacked[2],
                    'tension': unpacked[3],
                    'now_raw': unpacked[4],
                    'currentPositionX': unpacked[5],
                    'currentPositionY': unpacked[6],
                    'crc16': unpacked[7],
                }
                remaining = buffer[i+STATUS_STRUCT_SIZE:]
                return status, remaining
    return None, buffer[-(STATUS_STRUCT_SIZE-1):]

def main():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    sleep(2)
    response = ser.read(500)
    print(f"Received: {response}") # первый статус - после инициализации мк

    control = CommandControlByte(command_id=3, flag1=1, flag2=0, flag3=1, flag4=0)
    param1, param2, param3, param4  = 123, 456, 789, 12
    cmd = create_command(control, param1, param2, param3, param4)
    print(f"Sending command: {cmd.hex()}")
    ser.write(cmd)
    ser.flush()
    sleep(1)
    buffer = bytearray()
    while True:
        # Read one byte at a time for continuous data reading
        incoming = ser.read(1)
        if incoming:
            print(f"{incoming.hex()}",end='')
            buffer += incoming
            # Try to parse from buffer
            status, buffer = parse_status(buffer)
            if status:
                print("\nParsed status:", status)
                break  # or continue to keep reading more statuses
        else:
            # No byte received (timeout), can add sleep or continue
            pass

    ser.close()

if __name__ == "__main__":
    main()
