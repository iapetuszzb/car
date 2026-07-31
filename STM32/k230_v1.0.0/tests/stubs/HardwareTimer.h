#pragma once

#include <cstdint>

constexpr int HERTZ_FORMAT = 0;

class HardwareTimer {
public:
    explicit HardwareTimer(int) {}
    void setOverflow(uint32_t, int) {}
    void attachInterrupt(void (*)()) {}
    void resume() {}
};

