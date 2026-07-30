#include "encoder.h"

static volatile int32_t odom_signed_counts_a;
static volatile int32_t odom_signed_counts_b;
static volatile uint32_t odom_travel_counts_a;
static volatile uint32_t odom_travel_counts_b;

static uint32_t encoder_abs_i16(int16_t value)
{
    return (uint32_t)((value < 0) ? -(int32_t)value : (int32_t)value);
}

void ReadEncoder(GPIO_Regs *INVC_PORT, uint32_t INVC_PIN,
                 GPIO_Regs *GPIO_PORT, uint32_t GPIO_PIN,
                 volatile int *EncoderCount) {
    // 获取中断状态
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(INVC_PORT, INVC_PIN);

    if (status & INVC_PIN) {
        // 判断B相信号电平
        if (DL_GPIO_readPins(GPIO_PORT, GPIO_PIN)) {
            (*EncoderCount)++;  // 顺时针
        } else {
            (*EncoderCount)--;  // 逆时针
        }
        // 清除中断标志
        DL_GPIO_clearInterruptStatus(INVC_PORT, INVC_PIN);
    }
}

int ReadSpeed(volatile int *EncoderCount) {
    int Speed = *EncoderCount;
    *EncoderCount = 0;
    return Speed;
}

void Encoder_OdometryReset(void)
{
    uint32_t timer_irq_enabled = NVIC_GetEnableIRQ(TIMER_0_INST_INT_IRQN);

    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    odom_signed_counts_a = 0;
    odom_signed_counts_b = 0;
    odom_travel_counts_a = 0U;
    odom_travel_counts_b = 0U;
    if(timer_irq_enabled != 0U) {
        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    }
}

void Encoder_OdometryUpdate(int16_t delta_a, int16_t delta_b)
{
    odom_signed_counts_a += (int32_t)delta_a;
    odom_signed_counts_b += (int32_t)delta_b;
    odom_travel_counts_a += encoder_abs_i16(delta_a);
    odom_travel_counts_b += encoder_abs_i16(delta_b);
}

float Encoder_CountsToMm(float counts)
{
    return counts / ENCODER_COUNTS_PER_MM;
}

void Encoder_GetOdometry(encoder_odometry_t *odometry)
{
    uint32_t timer_irq_enabled;
    int32_t signed_a;
    int32_t signed_b;
    uint32_t travel_a;
    uint32_t travel_b;

    if(odometry == NULL) {
        return;
    }

    timer_irq_enabled = NVIC_GetEnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    signed_a = odom_signed_counts_a;
    signed_b = odom_signed_counts_b;
    travel_a = odom_travel_counts_a;
    travel_b = odom_travel_counts_b;
    if(timer_irq_enabled != 0U) {
        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    }

    odometry->signed_counts_a = signed_a;
    odometry->signed_counts_b = signed_b;
    odometry->travel_counts_a = travel_a;
    odometry->travel_counts_b = travel_b;
    odometry->distance_a_mm = Encoder_CountsToMm((float)travel_a);
    odometry->distance_b_mm = Encoder_CountsToMm((float)travel_b);
    odometry->distance_center_mm =
        (odometry->distance_a_mm + odometry->distance_b_mm) * 0.5f;
}
