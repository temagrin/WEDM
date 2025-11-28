#ifndef FIRMWARE_RING_BUFFER_H
#define FIRMWARE_RING_BUFFER_H

#include <cstdint>

#define COMMAND_BUFFER_SIZE 256
static constexpr std::size_t MASK = 255;

struct MotorCommand{
    uint8_t  ctrl_flags;     // сам байт-команда
    uint32_t stepsX;       // первый параметр
    uint32_t stepsY;       // второй параметр
    int32_t  speedX;       // третий параметр
    int32_t  speedY;       // четвертый параметр
};


class CommandRingBuffer{
public:
    bool push(uint8_t ctrl_flags, uint32_t stepsX, uint32_t stepsY, int32_t speedX, int32_t speedY);
    bool pop(MotorCommand& out);

    [[nodiscard]] inline std::size_t available() const {return capacity() - size();}

    [[nodiscard]] inline bool isEmpty() const { return count_ == 0; }
    [[nodiscard]] inline bool isFull()  const { return count_ == COMMAND_BUFFER_SIZE; }

    [[nodiscard]] inline std::size_t size() const { return count_; }
    [[nodiscard]] inline static std::size_t capacity() { return COMMAND_BUFFER_SIZE; }

private:
    MotorCommand buffer_[COMMAND_BUFFER_SIZE]{};
    std::size_t writeIndex_ = 0;
    std::size_t readIndex_  = 0;
    volatile std::size_t count_ = 0;
};


#endif //FIRMWARE_RING_BUFFER_H
