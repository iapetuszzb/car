#ifndef STEPPER_GIMBAL_H_
#define STEPPER_GIMBAL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STEPPER_GIMBAL_MOTOR_A = 0,  /* Lower yaw/pan axis. */
    STEPPER_GIMBAL_MOTOR_B = 1,  /* Upper pitch axis. */
} StepperGimbalMotor;

typedef enum {
    STEPPER_GIMBAL_DIR_NEGATIVE = -1,
    STEPPER_GIMBAL_DIR_POSITIVE = 1,
} StepperGimbalDirection;

typedef struct {
    volatile uint32_t target_steps;
    volatile uint32_t done_steps;
    volatile int32_t position_steps;
    volatile bool is_moving;
    int8_t direction;
} StepperGimbalState;

#define STEPPER_GIMBAL_STEPS_PER_REV     (3200U)
#define STEPPER_GIMBAL_ANGLE_PER_STEP    (0.1125f)

/*
 * D36A wiring used by this driver:
 * Motor A: ST1=PA26, DIR1=PB19, EN1=PB18
 * Motor B: ST2=PB24, DIR2=PA18, EN2=PA27
 * Voltage ADC: PA22 / ADC0_CH7, reserved for later sampling.
 * PA17/PA24 are no longer configured as buttons; the D36A manual control
 * interface only needs EN/DIR/ST for each motor plus ADC and common GND.
 */
void StepperGimbal_Init(void);
void StepperGimbal_Enable(StepperGimbalMotor motor, bool enable);
void StepperGimbal_SetHoldAfterMove(bool hold);

bool StepperGimbal_MoveSteps(StepperGimbalMotor motor, int32_t steps,
                             uint32_t step_hz);
bool StepperGimbal_MoveToSteps(StepperGimbalMotor motor, int32_t target_steps,
                               uint32_t step_hz);
bool StepperGimbal_SetVelocity(StepperGimbalMotor motor,
                               int32_t signed_step_hz);
bool StepperGimbal_MoveAngle(StepperGimbalMotor motor, float angle_deg,
                             uint32_t step_hz);

void StepperGimbal_Stop(StepperGimbalMotor motor);
void StepperGimbal_StopAll(void);
bool StepperGimbal_IsBusy(StepperGimbalMotor motor);
void StepperGimbal_Tick100us(void);
void StepperGimbal_Tick1ms(void);

int32_t StepperGimbal_GetPositionSteps(StepperGimbalMotor motor);
float StepperGimbal_GetAngleDeg(StepperGimbalMotor motor);
const StepperGimbalState *StepperGimbal_GetState(StepperGimbalMotor motor);

void StepperGimbal_TestSmallMove(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_GIMBAL_H_ */
