#ifndef FIRMWARE_RING_BUFFER_H
#define FIRMWARE_RING_BUFFER_H

#include <cstdint>

#define COMMAND_BUFFER_SIZE 256
static constexpr std::size_t MASK = 255;

struct MotorCommand{
    uint8_t  ctrl_flags;     // сам байт-команда
    uint32_t param1;       // первый параметр
    uint32_t param2;       // второй параметр
    int32_t  param3;       // третий параметр
    int32_t  param4;       // четвертый параметр
};


class CommandRingBuffer{
public:
    bool push(uint8_t ctrl_flags, uint32_t param1, uint32_t param2, int32_t param3, int32_t param4)
    {
        if (isFull())
            return false;
        MotorCommand mc{ctrl_flags, param1, param2, param3, param4};
        buffer_[writeIndex_] = mc;
        writeIndex_ = (writeIndex_ + 1) & MASK;  // вместо % N
        ++count_;
        return true;
    }

    bool pop(MotorCommand& out)
    {
        if (isEmpty())
            return false;

        out = buffer_[readIndex_];
        readIndex_  = (readIndex_  + 1) & MASK;
        --count_;
        return true;
    }

    [[nodiscard]] std::size_t available() const {
        return capacity() - size();
    }

    [[nodiscard]] bool isEmpty() const { return count_ == 0; }
    [[nodiscard]] bool isFull()  const { return count_ == COMMAND_BUFFER_SIZE; }

    [[nodiscard]] std::size_t size() const { return count_; }
    [[nodiscard]] static std::size_t capacity() { return COMMAND_BUFFER_SIZE; }

private:
    MotorCommand buffer_[COMMAND_BUFFER_SIZE];
    std::size_t writeIndex_ = 0;
    std::size_t readIndex_  = 0;
    volatile std::size_t count_ = 0;
};


#endif //FIRMWARE_RING_BUFFER_H
