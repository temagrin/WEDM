// pulse_generator.h
#ifndef PULSE_GENERATOR_H
#define PULSE_GENERATOR_H

#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

class PulseGenerator {
public:
    PulseGenerator(uint8_t pin);

    void setPulseWidth(uint32_t pulse_us);   // Длительность импульса, мкс
    void setPausePeriod(uint32_t pause_us);  // Период паузы, мкс
    void setFrequency(float freq_hz);         // Установить частоту Гц (перезапишет паузу)
    void enable(bool on);                      // Включить/выключить PWM

    inline uint32_t getPulseWidth() const { return _pulseWidth_us; }
    inline uint32_t getPausePeriod() const { return _pausePeriod_us; }
    inline float getFrequency() const { return _frequency; }
    inline bool isEnabled() const { return _enabled; }

private:
    uint8_t _pin;
    uint slice_num;
    uint chan;

    uint32_t _pulseWidth_us = 1;
    uint32_t _pausePeriod_us = 50;
    bool _enabled = false;
    float _frequency = 1000000.0f / (_pulseWidth_us + _pausePeriod_us);

    void updatePWM();
};
#endif // PULSE_GENERATOR_H