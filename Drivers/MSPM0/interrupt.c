#include "interrupt.h"
#include "ti_msp_dl_config.h"
#include "main.h"

#include "clock.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "IR_Module.h"
#include "stepper_gimbal.h"
#include "k210_face.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

volatile int EncoderCount_A, EncoderCount_B;
volatile int16_t Speed_A, Speed_B;
static encoder_quadrature_t encoder_decoder_a;
static encoder_quadrature_t encoder_decoder_b;

extern uint8_t turncount;
extern bool start, turnMark;
extern float TargetSpeed, TargetLine, basespeed;
extern pid_t pidMotorA, pidMotorB, pidLine;
extern volatile unsigned long tick_100us;

#define LINE_ACCEL_PER_TICK 0.04f
#define LINE_DECEL_PER_TICK 0.025f
#define LINE_APPROACH_SPEED 8.2f
#define LINE_DECEL_START_DISTANCE_MM 5000.0f
#define LINE_STOP_ARM_DISTANCE_MM 5000.0f

#define LINE_SPEED_BUTTON_STEP 0.2f
#define LINE_MIN_CRUISE_SPEED 8.5f
#define LINE_MAX_CRUISE_SPEED 12.0f
#define LINE_BUTTON_DEBOUNCE_TICKS 3U

#define LINE_SW1_PORT GPIOA
#define LINE_SW1_PIN DL_GPIO_PIN_27
#define LINE_SW1_IOMUX IOMUX_PINCM60
#define LINE_SW2_PORT GPIOB
#define LINE_SW2_PIN DL_GPIO_PIN_9
#define LINE_SW2_IOMUX IOMUX_PINCM26
#define LINE_SW3_PORT GPIOB
#define LINE_SW3_PIN DL_GPIO_PIN_24
#define LINE_SW3_IOMUX IOMUX_PINCM52
#define LINE_SW4_PORT GPIOA
#define LINE_SW4_PIN DL_GPIO_PIN_26
#define LINE_SW4_IOMUX IOMUX_PINCM59

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    IOMUX_PINCM iomux;
    uint8_t last_sample;
    uint8_t stable_state;
    uint8_t stable_count;
} line_speed_button_t;

static line_speed_button_t line_sw1 = {
    LINE_SW1_PORT, LINE_SW1_PIN, LINE_SW1_IOMUX, 1U, 1U, 0U
};
static line_speed_button_t line_sw2 = {
    LINE_SW2_PORT, LINE_SW2_PIN, LINE_SW2_IOMUX, 1U, 1U, 0U
};
static line_speed_button_t line_sw3 = {
    LINE_SW3_PORT, LINE_SW3_PIN, LINE_SW3_IOMUX, 1U, 1U, 0U
};
static line_speed_button_t line_sw4 = {
    LINE_SW4_PORT, LINE_SW4_PIN, LINE_SW4_IOMUX, 1U, 1U, 0U
};

static volatile bool line_run_timer_started = false;
static volatile bool line_run_timer_running = false;
static volatile unsigned long line_run_start_ms = 0UL;
static volatile unsigned long line_run_elapsed_ms = 0UL;

#if APP_ENABLE_SERVO_SWEEP
extern void Servo_Tick100us(void);
#endif
static void BoardButton_Update(void);
static void LineSpeed_Update(void);
static void LineSpeedButtons_Init(void);
static void LineSpeedButtons_Update(void);
static bool LineSpeedButton_UpdateOne(line_speed_button_t *button);
static void LineRunTimer_Start(void);
static void LineRunTimer_Stop(void);

static uint8_t LineSpeedButton_Read(const line_speed_button_t *button)
{
    return (DL_GPIO_readPins(button->port, button->pin) & button->pin) ?
           1U : 0U;
}

static void LineSpeedButton_InitOne(line_speed_button_t *button)
{
    uint8_t sample;

    DL_GPIO_initDigitalInputFeatures(
        button->iomux,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    sample = LineSpeedButton_Read(button);
    button->last_sample = sample;
    button->stable_state = sample;
    button->stable_count = 0U;
}

static void LineSpeedButtons_Init(void)
{
    /* These pins were formerly assigned to disabled vision/gimbal paths. */
    LineSpeedButton_InitOne(&line_sw1);
    LineSpeedButton_InitOne(&line_sw2);
    LineSpeedButton_InitOne(&line_sw3);
    LineSpeedButton_InitOne(&line_sw4);
}

static bool LineSpeedButton_UpdateOne(line_speed_button_t *button)
{
    uint8_t sample = LineSpeedButton_Read(button);

    if(sample == button->last_sample) {
        if(button->stable_count < LINE_BUTTON_DEBOUNCE_TICKS) {
            button->stable_count++;
        }
    } else {
        button->last_sample = sample;
        button->stable_count = 0U;
    }

    if((button->stable_count >= LINE_BUTTON_DEBOUNCE_TICKS) &&
       (sample != button->stable_state)) {
        button->stable_state = sample;
        return sample == 0U;
    }
    return false;
}

static void LineSpeedButtons_Update(void)
{
    if(LineSpeedButton_UpdateOne(&line_sw1)) {
        basespeed += LINE_SPEED_BUTTON_STEP;
        if(basespeed > LINE_MAX_CRUISE_SPEED) {
            basespeed = LINE_MAX_CRUISE_SPEED;
        }
    }

    if(LineSpeedButton_UpdateOne(&line_sw2)) {
        basespeed -= LINE_SPEED_BUTTON_STEP;
        if(basespeed < LINE_MIN_CRUISE_SPEED) {
            basespeed = LINE_MIN_CRUISE_SPEED;
        }
    }

    /* SW3 and SW4 are debounced and reserved for later functions. */
    (void)LineSpeedButton_UpdateOne(&line_sw3);
    (void)LineSpeedButton_UpdateOne(&line_sw4);
}

static void LineRunTimer_Start(void)
{
    line_run_start_ms = tick_ms;
    line_run_elapsed_ms = 0UL;
    line_run_timer_started = true;
    line_run_timer_running = true;
}

static void LineRunTimer_Stop(void)
{
    if(line_run_timer_running) {
        line_run_elapsed_ms = tick_ms - line_run_start_ms;
        line_run_timer_running = false;
    }
}

bool LineRunTimer_HasStarted(void)
{
    return line_run_timer_started;
}

bool LineRunTimer_IsRunning(void)
{
    return line_run_timer_running;
}

unsigned long LineRunTimer_GetElapsedMs(void)
{
    if(line_run_timer_running) {
        return tick_ms - line_run_start_ms;
    }
    return line_run_elapsed_ms;
}

void Interrupt_Init(void)
{
    LineSpeedButtons_Init();
    Encoder_QuadratureInit(&encoder_decoder_a,
                           GPIO_EncoderA_PORT,
                           GPIO_EncoderA_INVC_A_PIN,
                           GPIO_EncoderA_GPIO_A_PIN);
    Encoder_QuadratureInit(&encoder_decoder_b,
                           GPIO_EncoderB_PORT,
                           GPIO_EncoderB_INVC_B_PIN,
                           GPIO_EncoderB_GPIO_B_PIN);
    DL_GPIO_clearInterruptStatus(
        GPIO_EncoderA_PORT,
        GPIO_EncoderA_INVC_A_PIN | GPIO_EncoderA_GPIO_A_PIN);
    DL_GPIO_clearInterruptStatus(
        GPIO_EncoderB_PORT,
        GPIO_EncoderB_INVC_B_PIN | GPIO_EncoderB_GPIO_B_PIN);
    NVIC_SetPriority(GPIO_EncoderA_INT_IRQN, 1U);
    NVIC_SetPriority(GPIO_MULTIPLE_GPIOB_INT_IRQN, 1U);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 2U);
    NVIC_EnableIRQ(GPIO_EncoderA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void SysTick_Handler(void)
{
    static uint8_t tick_100us_count = 0U;

    tick_100us++;
#if APP_ENABLE_STEPPER_GIMBAL
    StepperGimbal_Tick100us();
#endif
#if APP_ENABLE_SERVO_SWEEP
    Servo_Tick100us();
#endif

    tick_100us_count++;
    if (tick_100us_count >= 10U) {
        tick_100us_count = 0U;
        tick_ms++;
#if APP_ENABLE_VISION_UART
        K210Face_Tick1ms();
#endif
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
        {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB,
                                                                UINT32_MAX);
            uint32_t encoder_status = status &
                (GPIO_EncoderB_INVC_B_PIN | GPIO_EncoderB_GPIO_B_PIN);

            if(status != 0U) {
                DL_GPIO_clearInterruptStatus(GPIOB, status);
            }
            if(encoder_status != 0U) {
                Encoder_QuadratureUpdate(&encoder_decoder_b,
                                         GPIO_EncoderB_PORT,
                                         GPIO_EncoderB_INVC_B_PIN,
                                         GPIO_EncoderB_GPIO_B_PIN,
                                         &EncoderCount_B);
            }
            break;
        }
        #endif

        #if defined GPIO_EncoderA_INT_IIDX
        case GPIO_EncoderA_INT_IIDX:
        {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(
                GPIO_EncoderA_PORT,
                GPIO_EncoderA_INVC_A_PIN | GPIO_EncoderA_GPIO_A_PIN);
            if(status != 0U) {
                DL_GPIO_clearInterruptStatus(GPIO_EncoderA_PORT, status);
                Encoder_QuadratureUpdate(&encoder_decoder_a,
                                         GPIO_EncoderA_PORT,
                                         GPIO_EncoderA_INVC_A_PIN,
                                         GPIO_EncoderA_GPIO_A_PIN,
                                         &EncoderCount_A);
            }
            break;
        }
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
            Encoder_OdometryUpdate(Speed_A, Speed_B);
            LineSpeedButtons_Update();

            if (!Bluetooth_MotionIsActive()) {
                BoardButton_Update();
            }

            if (Bluetooth_MotionControlTick10ms(Speed_A, Speed_B)) {
                /* Bluetooth motion owns the motors for this control tick. */
            } else if(start) {
                LineSpeed_Update();
                pid_control_line(TargetLine, TargetSpeed);
                if(PID_LineStopIsActive()) {
                    LineRunTimer_Stop();
                }
                if(PID_LineStopIsComplete()) {
                    start = false;
                    TargetSpeed = 0.0f;
                }
            } else {
                Load(0, 0);
            }
            break;

        default:
            break;
    }
}

static void LineSpeed_Update(void)
{
    float distance_mm = Encoder_GetDistanceCenterMm();
    float desired_speed = basespeed;

    if(distance_mm >= LINE_DECEL_START_DISTANCE_MM) {
        if(desired_speed > LINE_APPROACH_SPEED) {
            desired_speed = LINE_APPROACH_SPEED;
        }
    }

    PID_LineSetStopArmed(distance_mm >= LINE_STOP_ARM_DISTANCE_MM);

    if(TargetSpeed < desired_speed) {
        TargetSpeed += LINE_ACCEL_PER_TICK;
        if(TargetSpeed > desired_speed) {
            TargetSpeed = desired_speed;
        }
    } else if(TargetSpeed > desired_speed) {
        TargetSpeed -= LINE_DECEL_PER_TICK;
        if(TargetSpeed < desired_speed) {
            TargetSpeed = desired_speed;
        }
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
            PID_LineControlReset();
            Encoder_OdometryReset();
            turnMark = false;
            turncount = 0;
            TargetSpeed = 0.0f;
            LineRunTimer_Start();
            start = true;
            DL_GPIO_togglePins(GPIO_RGB_PORT, GPIO_RGB_USER_LED_2_PIN);
        }
    }
}
