#include "ring_buffer.h"



bool CommandRingBuffer::push(uint8_t ctrl_flags, uint32_t stepsX, uint32_t stepsY, int32_t speedX, int32_t speedY)
{
    if (isFull()) {
        return false;
    }
    MotorCommand mc{ctrl_flags, stepsX, stepsY, speedX, speedY};
    buffer_[writeIndex_] = mc;
    writeIndex_ = (writeIndex_ + 1) & MASK;
    ++count_;

    return true;
}

bool CommandRingBuffer::pop(MotorCommand& out) {
    if (isEmpty()) {
        return false;
    }
    out = buffer_[readIndex_];
    readIndex_  = (readIndex_  + 1) & MASK;
    --count_;

    return true;
}

