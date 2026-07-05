#include "motor.h"
#include "ti_msp_dl_config.h"

#define MAX_PWM 24000
#define MIN_PWM -24000

static int clamp_pwm(int pwm)
{
    if(pwm > MAX_PWM) return MAX_PWM;
    if(pwm < MIN_PWM) return MIN_PWM;
    return pwm;
}

void Motor_Init(void)
{
    DL_TimerG_startCounter(PWM_MOTORA_INST);
    DL_TimerG_startCounter(PWM_MOTORB_INST);
}

int abs(int x)
{
    if(x > 0) {
        return x;
    }
    return -x;
}

void Load(int motoA, int motoB)
{
    motoA = clamp_pwm(motoA);
    motoB = clamp_pwm(motoB);

    if(motoA < 0) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTORA_INST, abs(motoA), GPIO_PWM_MOTORA_C0_IDX);
        DL_TimerA_setCaptureCompareValue(PWM_MOTORA_INST, 0, GPIO_PWM_MOTORA_C1_IDX);
    } else if(motoA > 0) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTORA_INST, 0, GPIO_PWM_MOTORA_C0_IDX);
        DL_Timer_setCaptureCompareValue(PWM_MOTORA_INST, abs(motoA), GPIO_PWM_MOTORA_C1_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_MOTORA_INST, 0, GPIO_PWM_MOTORA_C0_IDX);
        DL_TimerA_setCaptureCompareValue(PWM_MOTORA_INST, 0, GPIO_PWM_MOTORA_C1_IDX);
    }

    /* Motor B is the left wheel BO1/BO2 output, reversed for this chassis. */
    if(motoB < 0) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTORB_INST, 0, GPIO_PWM_MOTORB_C0_IDX);
        DL_TimerA_setCaptureCompareValue(PWM_MOTORB_INST, abs(motoB), GPIO_PWM_MOTORB_C1_IDX);
    } else if(motoB > 0) {
        DL_TimerA_setCaptureCompareValue(PWM_MOTORB_INST, abs(motoB), GPIO_PWM_MOTORB_C0_IDX);
        DL_TimerA_setCaptureCompareValue(PWM_MOTORB_INST, 0, GPIO_PWM_MOTORB_C1_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_MOTORB_INST, 0, GPIO_PWM_MOTORB_C0_IDX);
        DL_TimerA_setCaptureCompareValue(PWM_MOTORB_INST, 0, GPIO_PWM_MOTORB_C1_IDX);
    }
}

void Limit(int *MotoA, int *MotoB)
{
    *MotoA = clamp_pwm(*MotoA);
    *MotoB = clamp_pwm(*MotoB);
}
