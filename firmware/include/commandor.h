#ifndef FIRMWARE_COMMANDOR_H
#define FIRMWARE_COMMANDOR_H

#include <pico/types.h>
#include "pulse_generator.h"
#include "motor_controller.h"
#include "current_sensor.h"

struct CommandControlByte {
    uint8_t commandId : 4;
    uint8_t flag1 : 1;
    uint8_t flag2 : 1;
    uint8_t flag3 : 1;
    uint8_t flag4 : 1;

} __attribute__((packed));

struct Command {
    uint16_t start_marker;      // 2
    CommandControlByte control; // 1
    uint32_t param1;            // 4
    uint32_t param2;            // 4
    uint32_t param3;            // 4
    uint32_t param4;            // 4
    int32_t commandUID;         // 4
    uint16_t crc16;             // 2
} __attribute__((packed));


struct Status {
    uint16_t start_marker;       // 2 байта
    uint16_t bufferFilling;      // 2 байта
    uint16_t amperage;           // 2 байта
    int16_t tension;             // 2 байта
    absolute_time_t now;         // 8 байт
    int32_t currentPositionX;    // 4 байта
    int32_t currentPositionY;    // 4 байта
    int32_t lastCommandUID;      // 4 байта
    uint16_t crc16;              // 2 байта
} __attribute__((packed));


#define SYNC_MARKER 0xABCD
#define COMMAND_PACKET_SIZE sizeof(Command)
#define COMMAND_DATA_SIZE (COMMAND_PACKET_SIZE - sizeof(uint16_t))
#define RESPONSE_PACKET_SIZE sizeof(Status)
#define RESPONSE_DATA_SIZE (sizeof(Status) - sizeof(uint16_t))

class CommandManager{
public:
    CommandManager(StepperMotorController* motorController, CurrentSensor* currentSensor, PulseGenerator* pulseGenerator);
    void sendStatus();
    void parseCommand();

private:
    StepperMotorController* motorController;
    CurrentSensor* currentSensor;
    PulseGenerator* pulseGenerator;
    Status status;
    uint8_t receiveBuffer[COMMAND_PACKET_SIZE];
    uint16_t calculate_crc16(const uint8_t *data, size_t length);
    void processReceivedCommand(Command *cmd);
};
#endif //FIRMWARE_COMMANDOR_H
