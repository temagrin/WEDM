#ifndef FIRMWARE_RING_BUFFER_H
#define FIRMWARE_RING_BUFFER_H

#include <cstdint>

#define COMMAND_BUFFER_SIZE 256
static constexpr std::size_t MASK = 255;

struct MotorCommand {
    uint8_t ctrlFlags;
    uint32_t stepsX;
    uint32_t stepsY;
    uint32_t stepIntervalX;
    uint32_t stepIntervalY;
    uint32_t errorIncrementX;
    uint32_t errorIncrementY;
};


class CommandRingBuffer {
public:
    bool push(uint8_t ctrlFlags, uint32_t stepsX, uint32_t stepsY,
              uint32_t stepIntervalX, uint32_t stepIntervalY,
              uint32_t errorIncrementX, uint32_t errorIncrementY);

    bool pop(MotorCommand &out);

    [[nodiscard]] std::size_t available() const { return capacity() - size(); }

    [[nodiscard]] bool isEmpty() const { return count_ == 0; }
    [[nodiscard]] bool isFull() const { return count_ == COMMAND_BUFFER_SIZE; }

    [[nodiscard]] std::size_t size() const { return count_; }
    [[nodiscard]] static std::size_t capacity() { return COMMAND_BUFFER_SIZE; }

private:
    MotorCommand buffer_[COMMAND_BUFFER_SIZE]{};
    std::size_t writeIndex_ = 0;
    std::size_t readIndex_ = 0;
    volatile std::size_t count_ = 0;
};


#endif //FIRMWARE_RING_BUFFER_H
