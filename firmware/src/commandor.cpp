#include <cstring>
#include <cstdio>
#include <pico/time.h>
#include "commandor.h"
#include "tusb.h"

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
    status.now = get_absolute_time();
    status.amperage = currentSensor->getCurrent();
    status.tension = 69;
    status.currentPositionX = motorController->getCurrentPositionX();
    status.currentPositionY = motorController->getCurrentPositionY();
    uint16_t crc = calculate_crc16((uint8_t*)&status,RESPONSE_DATA_SIZE);
    status.crc16 = crc;
    tud_cdc_write((const uint8_t *)&status, RESPONSE_PACKET_SIZE);
    tud_cdc_write_flush();
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
    if (tud_cdc_available() > 0) {
        uint8_t byte;
        tud_cdc_read(&byte, 1);
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
    status.lastCommandUID = cmd->commandUID;
    switch (cmd->control.commandId){
        case 1: // команда выключить двигателей XYAB
            StepperMotorController::setPowerXY(cmd->control.flag1);
            StepperMotorController::setPowerA(cmd->control.flag2);
            StepperMotorController::setPowerB(cmd->control.flag3);
            break;

        case 2: // команда задания Включить и задать скорости вращения двигателей XYAB
            StepperMotorController::setPowerXY(true);
            StepperMotorController::setPowerA(true);
            StepperMotorController::setPowerB(true);
            motorController->setSpeedX(cmd->control.flag1 ? (int32_t)cmd->param1: -(int32_t)cmd->param1);
            motorController->setSpeedY(cmd->control.flag2 ? (int32_t)cmd->param2: -(int32_t)cmd->param2);
            motorController->setSpeedA(cmd->control.flag3 ? (int32_t)cmd->param3: -(int32_t)cmd->param3);
            motorController->setSpeedB(cmd->control.flag4 ? (int32_t)cmd->param4: -(int32_t)cmd->param4);
            break;
        default:
            break;
    }

}


