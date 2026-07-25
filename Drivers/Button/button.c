#include "button.h"

/*
 * Generic external-button handling is disabled for now.
 *
 * The only active button path should be the on-board PB21 button handled by
 * BoardButton_Update() in Drivers/MSPM0/interrupt.c. Keeping these stubs lets
 * old code compile without reading or reserving any extra button pins.
 */

void Button_Init(button_t *button, GPIO_Regs *PORT, uint32_t PIN)
{
    (void)button;
    (void)PORT;
    (void)PIN;
}

uint8_t Button_ReadState(button_t *button)
{
    (void)button;
    return 0U;
}

void Button_Update(button_t *button)
{
    (void)button;
}

void Button_UpdateFromInterrupt(button_t *button, uint8_t pinState)
{
    (void)button;
    (void)pinState;
}
