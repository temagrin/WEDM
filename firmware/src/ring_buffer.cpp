#include "ring_buffer.h"


bool CommandRingBuffer::push(const uint8_t ctrlFlags, const uint32_t stepsX, const uint32_t stepsY,
                             const uint32_t stepInterval, const uint32_t errorIncrement) {
    if (isFull()) {
        return false;
    }
    const MotorCommand mc{ctrlFlags, stepsX, stepsY, stepInterval, errorIncrement};
    buffer_[writeIndex_] = mc;
    writeIndex_ = (writeIndex_ + 1) & MASK;
    ++count_;
    return true;
}

bool CommandRingBuffer::pop(MotorCommand &out) {
    if (isEmpty()) {
        return false;
    }
    out = buffer_[readIndex_];
    readIndex_ = (readIndex_ + 1) & MASK;
    --count_;
    return true;
}
