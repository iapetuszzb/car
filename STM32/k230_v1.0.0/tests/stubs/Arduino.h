#pragma once

#include <cstddef>
#include <cstdint>

using PinName = int;

constexpr int PA9 = 9;
constexpr int PA10 = 10;
constexpr int PB6 = 106;
constexpr int PB7 = 107;
constexpr int PB8 = 108;
constexpr int PB10 = 110;
constexpr int PB11 = 111;
constexpr int TIM2 = 2;

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OUTPUT = 1;

constexpr std::size_t ARDUINO_STUB_PIN_CAPACITY = 256;
inline int arduino_stub_pin_value[ARDUINO_STUB_PIN_CAPACITY]{};
inline uint32_t arduino_stub_pin_write_count[ARDUINO_STUB_PIN_CAPACITY]{};

inline void arduinoStubResetPins() {
    for (std::size_t i = 0; i < ARDUINO_STUB_PIN_CAPACITY; ++i) {
        arduino_stub_pin_value[i] = LOW;
        arduino_stub_pin_write_count[i] = 0;
    }
}

inline int arduinoStubPinValue(PinName pin) {
    return (pin >= 0 && static_cast<std::size_t>(pin) <
                            ARDUINO_STUB_PIN_CAPACITY)
        ? arduino_stub_pin_value[pin]
        : LOW;
}

inline uint32_t arduinoStubPinWriteCount(PinName pin) {
    return (pin >= 0 && static_cast<std::size_t>(pin) <
                            ARDUINO_STUB_PIN_CAPACITY)
        ? arduino_stub_pin_write_count[pin]
        : 0;
}

inline void pinMode(PinName, int) {}
inline void digitalWrite(PinName pin, int value) {
    if (pin < 0 || static_cast<std::size_t>(pin) >=
                       ARDUINO_STUB_PIN_CAPACITY) {
        return;
    }
    arduino_stub_pin_value[pin] = value;
    ++arduino_stub_pin_write_count[pin];
}
inline void noInterrupts() {}
inline void interrupts() {}
inline void delay(uint32_t) {}
inline uint32_t millis() { return 0; }

class HardwareSerial {
public:
    HardwareSerial(PinName, PinName) {}
    void begin(uint32_t) {}
    int available() const { return 0; }
    int read() { return -1; }

    template <typename T>
    void print(const T&) {}

    template <typename T>
    void print(const T&, int) {}

    void println() {}

    template <typename T>
    void println(const T&) {}
};
