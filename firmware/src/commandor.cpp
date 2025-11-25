#include <pico/error.h>
#include <pico/stdio.h>
#include <cstring>
#include <cstdio>
#include <pico/time.h>
#include "commandor.h"


CommandManager::CommandManager(
        StepperMotorController* _motorController,
        CurrentSensor* _currentSensor,
        PulseGenerator* _pulseGenerator):
                               motorController(_motorController),
                               currentSensor(_currentSensor),
                               pulseGenerator(_pulseGenerator) {
    status.start_marker = SYNC_MARKER;
}

void CommandManager::sendStatus() {
//    printf("\n%d %d %d %d\n", RESPONSE_PACKET_SIZE, sizeof(uint16_t), sizeof(int16_t), sizeof(int32_t));
    status.now = get_absolute_time();
    status.amperage = currentSensor->getCurrent();

    uint16_t crc = calculate_crc16((uint8_t*)&status,RESPONSE_DATA_SIZE);
    status.crc16 = crc;
    fwrite((const void*)&status, RESPONSE_PACKET_SIZE, 1, stdout);
    fflush(stdout);
}

uint16_t CommandManager::calculate_crc16(const uint8_t *data, size_t length) {
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

void CommandManager::parseCommand() {
    int byte = getchar_timeout_us(0);
    if (byte != PICO_ERROR_TIMEOUT) {
        memmove(receiveBuffer, receiveBuffer + 1, COMMAND_PACKET_SIZE - 1);
        receiveBuffer[COMMAND_PACKET_SIZE - 1] = (uint8_t)byte;
        auto *cmd = (Command *)receiveBuffer;
        if (cmd->start_marker == SYNC_MARKER) {
            uint16_t calculated_crc = calculate_crc16((uint8_t*)receiveBuffer, COMMAND_DATA_SIZE);
            if (calculated_crc == cmd->crc16) {
                Command actual_command{};
                memcpy(&actual_command, receiveBuffer, COMMAND_PACKET_SIZE);
                processReceivedCommand(&actual_command);
               memset(receiveBuffer, 0, COMMAND_PACKET_SIZE);
            }
        }
    }
}

void CommandManager::processReceivedCommand(Command *cmd) {
    status.currentPositionX = (int32_t)cmd->param1;
    status.currentPositionY = (int32_t)cmd->param2;
    sendStatus();
}


