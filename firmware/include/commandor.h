#ifndef FIRMWARE_COMMANDOR_H
#define FIRMWARE_COMMANDOR_H

#include "tusb.h"
#include <cstdio>
#include "pulse_generator.h"
#include "motor_controller.h"
#include "current_sensor.h"

#define SYNC_MARKER 0xABCD
#define MAX_COMMANDS_IN_PACKET 16

struct PacketHeader {
    uint16_t start_mark;      // 2 байта стартовый маркер
    uint16_t seq_id;          // 2 байта идентификатор пачки
    uint8_t  size_reserved;   // 4 бита размер (0-15), 4 бита зарезервировано
} __attribute__((packed)) ;

struct Command {
    uint8_t  cmd_id;          // 1 байт ID команды
    uint8_t  ctrl_flags;      // 1 байт контрольные флаги
    uint32_t param1;          // 4 байта
    uint32_t param2;          // 4 байта
    int32_t param3;          // 4 байта
    int32_t param4;          // 4 байта
} __attribute__((packed));

struct Packet {
    PacketHeader header;
    Command      commands[MAX_COMMANDS_IN_PACKET]; // максимум 16 команд, используем header.size_reserved
    int16_t      crc;          // 2 байта CRC
} __attribute__((packed));


struct Status {
    uint16_t    start_marker;       // 2 байта
    uint16_t    stepsBufferFree;    // 2 байта   - свободно в буфере планировщика шагов
    uint16_t    amperage;           // 2 байта   - ток ( информационно для демонстрации )  - коррекция на стороне МК
    int16_t     tension;            // 2 байта   - натяжение ( информационно для демонстрации ) - коррекция на стороне МК
    int32_t     currentPositionX;   // 4 байта   - положение по X ( информационно для демонстрации )
    int32_t     currentPositionY;   // 4 байта   - положение по Y ( информационно для демонстрации )
    uint16_t    ack_mask;           // 2 байта   - FF = вся пачка из 8ми командами с seq_id принята
    uint16_t    nak_mask;           // 2 байта   - FF = вся пачка из 8ми командами с seq_id не принята
    uint16_t    seq_id;             // 2 байта   - seq_id индификатор последней обработанной пачки
    uint16_t    crc;                // 2 байта
} __attribute__((packed)); // статусная пачка 24 байта



class CommandManager{
public:
    CommandManager(StepperMotorController& motorController, CurrentSensor& currentSensor, PulseGenerator& pulseGenerator);
    void sendStatus();
    void updateRX();

private:
    StepperMotorController& motorController;
    CurrentSensor& currentSensor;
    PulseGenerator& pulseGenerator;
    Status status{};
    size_t parseCommand();
    void processReceivedPacket(Packet *packet);
};
#endif //FIRMWARE_COMMANDOR_H
