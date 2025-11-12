#include "pulse_generator.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


PulseGenerator::PulseGenerator(uint8_t pin) : _pin(pin) {
    pinMode(_pin, OUTPUT);
    gpio_set_function(_pin, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(_pin);
    chan = pwm_gpio_to_channel(_pin);
    updatePWM();
}

void PulseGenerator::setPulseWidth(uint32_t pulse_us) {
    if (pulse_us < 1) pulse_us = 1;
    if (pulse_us > 65535) pulse_us = 65535;
    _pulseWidth_us = pulse_us;
    _frequency = 1000000.0f / (_pulseWidth_us + _pausePeriod_us);
    if(_enabled) updatePWM();
}

void PulseGenerator::setPausePeriod(uint32_t pause_us) {
    if (pause_us < 1) pause_us = 1;
    if (pause_us > 65535) pause_us = 65535;
    _pausePeriod_us = pause_us;
    _frequency = 1000000.0f / (_pulseWidth_us + _pausePeriod_us);
    if(_enabled) updatePWM();
}

void PulseGenerator::setFrequency(float freq_hz) {
    if (freq_hz <= 0) return;
    _frequency = freq_hz;
    uint32_t period_us = uint32_t(1000000.0f / _frequency);
    if(period_us > _pulseWidth_us)
        _pausePeriod_us = period_us - _pulseWidth_us;
    else
        _pausePeriod_us = 1;
    if(_enabled) updatePWM();
}

void PulseGenerator::enable(bool on) {
    _enabled = on;
    if (_enabled) {
        pwm_set_enabled(slice_num, true);
    } else {
        pwm_set_enabled(slice_num, false);
        digitalWrite(_pin, LOW);
    }
}

uint32_t PulseGenerator::getPulseWidth() const {
    return _pulseWidth_us;
}

uint32_t PulseGenerator::getPausePeriod() const {
    return _pausePeriod_us;
}

float PulseGenerator::getFrequency() const {
    return _frequency;
}

bool PulseGenerator::isEnabled() const {
    return _enabled;
}

void PulseGenerator::updatePWM() {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t wrap = (sys_clk / 1000000) * (_pulseWidth_us + _pausePeriod_us);
    if (wrap > 65535) wrap = 65535;
    pwm_set_wrap(slice_num, wrap);
    uint16_t level = uint16_t((uint64_t)wrap * _pulseWidth_us / (_pulseWidth_us + _pausePeriod_us));
    pwm_set_chan_level(slice_num, chan, level);
    pwm_set_enabled(slice_num, true);
}