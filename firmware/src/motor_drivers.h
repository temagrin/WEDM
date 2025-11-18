// motor_drivers.h
#ifndef MOTOR_DRIVERS_H
#define MOTOR_DRIVERS_H
#include "pico/stdlib.h"

struct MotorDriver {
    bool invert_step_pin = false;
    bool invert_dir_pin = false;
    bool invert_en_pin = false;
    uint8_t step_pulse_duration = 1;
};

inline const MotorDriver A4988 =  {false,   false,  false,  1};
inline const MotorDriver LV8729 = {false,   false,   true,  2};

#endif // MOTOR_DRIVERS_H