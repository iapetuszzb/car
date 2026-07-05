#include "interrupt.h"
#include "ti_msp_dl_config.h"

#include "clock.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "IR_Module.h"
#include "stepper_gimbal.h"
#include "k210_face.h"
#include <stdbool.h>
#include <stdint.h>

volatile int EncoderCount_A, EncoderCount_B;
volatile int16_t Speed_A, Speed_B;

extern uint8_t turncount;
extern bool start, turnMark;
extern float TargetSpeed, TargetLine, basespeed;
extern pid_t pidMotorA, pidMotorB, pidLine;

static void BoardButton_Update(void);

void Interrupt_Init(void)
{
    NVIC_EnableIRQ(GPIO_EncoderA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void SysTick_Handler(void)
{
    static uint8_t tick_100us_count = 0U;

    StepperGimbal_Tick100us();

    tick_100us_count++;
    if (tick_100us_count >= 10U) {
        tick_100us_count = 0U;
        tick_ms++;
        K210Face_Tick1ms();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
        {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB,
                                                                GPIO_EncoderB_INVC_B_PIN);
            if (status & GPIO_EncoderB_INVC_B_PIN) {
                ReadEncoder(GPIO_EncoderB_PORT, GPIO_EncoderB_INVC_B_PIN,
                            GPIO_EncoderB_PORT, GPIO_EncoderB_GPIO_B_PIN,
                            &EncoderCount_B);
            }
            break;
        }
        #endif

        #if defined GPIO_EncoderA_INT_IIDX
        case GPIO_EncoderA_INT_IIDX:
            ReadEncoder(GPIO_EncoderA_PORT, GPIO_EncoderA_INVC_A_PIN,
                        GPIO_EncoderA_PORT, GPIO_EncoderA_GPIO_A_PIN,
                        &EncoderCount_A);
            break;
        #endif

        default:
            break;
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            Speed_A = ReadSpeed(&EncoderCount_A);
            Speed_B = ReadSpeed(&EncoderCount_B);

            BoardButton_Update();

            if(start) {
                pid_control_line(TargetLine, TargetSpeed);
            } else {
                Load(0, 0);
            }
            break;

        default:
            break;
    }
}

static void BoardButton_Update(void)
{
    static uint8_t last_sample = 1;
    static uint8_t stable_state = 1;
    static uint8_t stable_count = 0;

    uint8_t sample = (DL_GPIO_readPins(GPIO_BUTTON_B_PORT,
                                       GPIO_BUTTON_B_USER_BUTTON_1_PIN) &
                      GPIO_BUTTON_B_USER_BUTTON_1_PIN) ? 1 : 0;

    if(sample == last_sample) {
        if(stable_count < 3) {
            stable_count++;
        }
    } else {
        last_sample = sample;
        stable_count = 0;
    }

    if((stable_count >= 3) && (sample != stable_state)) {
        stable_state = sample;

        if((stable_state == 0) && (start == false)) {
            PID_Reset(&pidMotorA);
            PID_Reset(&pidMotorB);
            PID_Reset(&pidLine);
            turnMark = false;
            turncount = 0;
            TargetSpeed = basespeed;
            start = true;
            DL_GPIO_togglePins(GPIO_RGB_PORT, GPIO_RGB_USER_LED_2_PIN);
        }
    }
}
