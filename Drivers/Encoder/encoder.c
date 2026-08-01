#include "encoder.h"

static volatile int32_t odom_signed_counts_a;
static volatile int32_t odom_signed_counts_b;
static volatile uint32_t odom_travel_counts_a;
static volatile uint32_t odom_travel_counts_b;

/* State is (phase A << 1) | phase B. Invalid two-bit jumps count as zero. */
static const int8_t quadrature_delta[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint32_t encoder_abs_i16(int16_t value)
{
    return (uint32_t)((value < 0) ? -(int32_t)value : (int32_t)value);
}

static uint8_t encoder_read_state(GPIO_Regs *port,
                                  uint32_t phase_a_pin,
                                  uint32_t phase_b_pin)
{
    uint32_t pins = DL_GPIO_readPins(port, phase_a_pin | phase_b_pin);
    uint8_t phase_a = (pins & phase_a_pin) ? 1U : 0U;
    uint8_t phase_b = (pins & phase_b_pin) ? 1U : 0U;

    return (uint8_t)((phase_a << 1U) | phase_b);
}

void Encoder_QuadratureInit(encoder_quadrature_t *decoder,
                            GPIO_Regs *port,
                            uint32_t phase_a_pin,
                            uint32_t phase_b_pin)
{
    if(decoder == NULL) {
        return;
    }
    decoder->previous_state =
        encoder_read_state(port, phase_a_pin, phase_b_pin);
}

void Encoder_QuadratureUpdate(encoder_quadrature_t *decoder,
                              GPIO_Regs *port,
                              uint32_t phase_a_pin,
                              uint32_t phase_b_pin,
                              volatile int *encoder_count)
{
    uint8_t current_state;
    uint8_t transition;

    if((decoder == NULL) || (encoder_count == NULL)) {
        return;
    }

    current_state = encoder_read_state(port, phase_a_pin, phase_b_pin);
    transition = (uint8_t)((decoder->previous_state << 2U) | current_state);
    *encoder_count += quadrature_delta[transition];
    decoder->previous_state = current_state;
}

int ReadSpeed(volatile int *EncoderCount)
{
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

float Encoder_GetDistanceCenterMm(void)
{
    float distance_a_mm = Encoder_CountsToMm((float)odom_travel_counts_a);
    float distance_b_mm = Encoder_CountsToMm((float)odom_travel_counts_b);

    return (distance_a_mm + distance_b_mm) * 0.5f;
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
