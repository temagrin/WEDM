#include <cstring>
#include <cstdio>
#include "commandor.h"
#include "tusb.h"

static const uint8_t CRC_SIZE = sizeof(uint16_t); // размер crc
static const uint8_t STATUS_PACKET_SIZE = sizeof(Status); // размер статусного пакета
static const uint8_t STATUS_DATA_SIZE   = STATUS_PACKET_SIZE - CRC_SIZE; //размер статусных данных
static const uint16_t PACKET_HEADER_SIZE = sizeof(PacketHeader); // размер одной команды
static const uint16_t COMMAND_SIZE = sizeof(Command); // размер одной команды
static const uint16_t MIN_PACKET_SIZE = PACKET_HEADER_SIZE + CRC_SIZE; // заголовок + crc
static const uint16_t MAX_PACKET_SIZE = MIN_PACKET_SIZE + (MAX_COMMANDS_IN_PACKET * COMMAND_SIZE);
static const uint16_t MAX_BUFFER_SIZE = 3*MAX_PACKET_SIZE;

static uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }}}
    return crc;
}

CommandManager::CommandManager(
        StepperMotorController &_motorController,
        CurrentSensor &_currentSensor,
        PulseGenerator &_pulseGenerator,
        CommandRingBuffer &queue) :
        motorController(_motorController),
        currentSensor(_currentSensor),
        pulseGenerator(_pulseGenerator),
        queue(queue) {
    status.start_marker = SYNC_MARKER;
}


void CommandManager::sendStatus() {
    status.stepsBufferFree = queue.available();
    status.amperage = currentSensor.getCurrent()>>4;
    status.tension = 69;
    status.currentPositionX = motorController.getCurrentPositionX();
    status.currentPositionY = motorController.getCurrentPositionY();
    uint16_t crc = calculate_crc16((uint8_t*)&status,STATUS_DATA_SIZE);
    status.crc = crc;
    tud_cdc_write((const uint8_t *)&status, STATUS_PACKET_SIZE);
    tud_cdc_write_flush();
}


uint8_t receiveBuffer[MAX_PACKET_SIZE];
size_t buf_len = 0;

void CommandManager::updateRX() {
    uint8_t new_bytes = tud_cdc_available();
    if (new_bytes == 0) return;
    if (buf_len + new_bytes > MAX_BUFFER_SIZE) {
        uint16_t discard_bytes = buf_len + new_bytes - MAX_BUFFER_SIZE;
        memmove(receiveBuffer, receiveBuffer + discard_bytes, buf_len - discard_bytes);
        buf_len -= discard_bytes;
    }
    tud_cdc_read(&receiveBuffer[buf_len], new_bytes);
    buf_len += new_bytes;
    if (buf_len>=MIN_PACKET_SIZE) {
        size_t offset = parseCommand();
        if (offset == 0) return;
        buf_len -= offset;
        memmove(receiveBuffer, receiveBuffer + offset, buf_len);
    }
}

size_t CommandManager::parseCommand() {
    // Ищем start_mark (0xCDAB little-endian)
    if (*(uint16_t*)receiveBuffer != SYNC_MARKER) return 1;

    // Парсим заголовок
    uint8_t size_reserved = receiveBuffer[4];
    uint8_t num_cmds = size_reserved & 0x0F;
    size_t pkt_size = PACKET_HEADER_SIZE + (num_cmds * COMMAND_SIZE) + CRC_SIZE;

    if (pkt_size > buf_len) {
        return 0;
    }

    size_t commandsSize = num_cmds*COMMAND_SIZE;
    size_t payloadSize = PACKET_HEADER_SIZE + commandsSize;
    // Проверяем CRC
    uint16_t recv_crc = *(uint16_t*)(receiveBuffer + payloadSize);
    uint16_t calc_crc = calculate_crc16((uint8_t*)&receiveBuffer,payloadSize);
    if (calc_crc != recv_crc) {
        return 2;
    }
    Packet packet = {
            .header = {0},
            .crc = 0};
    memset(packet.commands, 0, sizeof(packet.commands));

    // Копируем заголовок
    memcpy(&packet.header, receiveBuffer, sizeof(PacketHeader));

    // Копируем ТОЛЬКО реальные команды (0-16)
    size_t commands_offset = sizeof(PacketHeader);
    memcpy(packet.commands, receiveBuffer + commands_offset, num_cmds * sizeof(Command));

    // Копируем CRC
    memcpy(&packet.crc, receiveBuffer + payloadSize, sizeof(int16_t));

    processReceivedPacket(&packet);
    return payloadSize+CRC_SIZE;
}

void CommandManager::processReceivedPacket(Packet *packet) {
    status.seq_id = packet->header.seq_id;
    uint8_t num_cmds = packet->header.size_reserved & 0x0F;
//    uint8_t packet_flags = packet->header.size_reserved >> 4;
//
    status.ack_mask = 0;
    status.nak_mask = (1u << num_cmds) - 1;

    auto process_cmd = [&](uint8_t i, auto&& handler) {
        if (handler()) {
            status.ack_mask |= (1u << i);
            status.nak_mask &= ~(1u << i);
        }
    };
    for (uint8_t i = 0; i<num_cmds; i++) {
        const auto& cmd = packet->commands[i];
        sleep_us(10);
        switch (cmd.cmd_id) {
            case 1: // команда 1 - добавить в очередь на исполнения шагов
                process_cmd(i, [&]{ return queue.push(cmd.ctrl_flags, cmd.param1, cmd.param2, cmd.param3, cmd.param4);});
                break;
            case 2: // команда 2 - включить немедленно моторы по указаным флагам ( flag 1 - XY, flag 2 - A, flag 3 - B
                process_cmd(i, [&]{ return motorController.commonControl(cmd.ctrl_flags);});
                break;
            default:
                break;
        }
    }
}
