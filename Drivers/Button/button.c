#include "button.h"

#include "clock.h"

#define SHORT_PRESS_TIME 500    // 短按：>500ms
#define LONG_PRESS_TIME 1000    // 长按：>3000ms
#define DEBOUNCE_TIME 20        // 去抖动时间

extern volatile unsigned long tick_ms;

void Button_Init(button_t *button, GPIO_Regs *PORT, uint32_t PIN) {
    button->ButtonPort = PORT;
    button->ButtonPin = PIN;
    button->ButtonEvent = 0;
    button->isPressed = 0;
    button->lastPinState = 0;//默认上拉
    button->pressTick = 0;
}

// 非阻塞读取事件（读取后自动清除）
uint8_t Button_ReadState(button_t *button) {
    uint8_t evt = button->ButtonEvent;
    button->ButtonEvent = 0; // 读取后清除
    return evt;
}

// 轮询更新或许按钮状态（推荐10ms周期）
void Button_Update(button_t *button) {
    uint32_t val = DL_GPIO_readPins(button->ButtonPort, button->ButtonPin);
    uint8_t pinState = (val & button->ButtonPin) ? 1 : 0;

    if (pinState != button->lastPinState) {
        // 状态变化，开始去抖
        static unsigned long debounceTick = 0;
        if (tick_ms - debounceTick >= DEBOUNCE_TIME) {
            debounceTick = tick_ms;

            // 按键按下（低电平）
            if (pinState == 0 && button->isPressed == 0) {
                button->isPressed = 1;
                button->pressTick = tick_ms;
            }

            // 按键释放（高电平）
            else if (pinState == 1 && button->isPressed == 1) {
                unsigned long pressDuration = tick_ms - button->pressTick;

                if (pressDuration >= LONG_PRESS_TIME) {
                    button->ButtonEvent = 2; // 长按
                } else if (pressDuration >= SHORT_PRESS_TIME) {
                    button->ButtonEvent = 1; // 短按
                }
                button->isPressed = 0;
            }
        }
    }

    button->lastPinState = pinState;
}


// 基于中断获取按钮状态
void Button_UpdateFromInterrupt(button_t *button, uint8_t pinState)
{
    if (pinState == 0) { // 按键按下（下降沿中断触发）
        button->isPressed = 1;
        button->pressTick = tick_ms;
    } else if (pinState == 1 && button->isPressed == 1) { // 按键松开（上升沿中断触发）
        unsigned long pressDuration = tick_ms - button->pressTick;

        if (pressDuration >= LONG_PRESS_TIME) {
            button->ButtonEvent = 2; // 长按
        } else if (pressDuration >= SHORT_PRESS_TIME) {
            button->ButtonEvent = 1; // 短按
        }

        button->isPressed = 0;
    }
}
