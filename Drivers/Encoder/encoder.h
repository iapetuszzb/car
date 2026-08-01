#ifndef _ENCODER_H
#define _ENCODER_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* Both edges of both quadrature phases are decoded: 13 lines become
 * 52 counts per motor revolution before the 30:1 gearbox. */
#define ENCODER_LINES_PER_MOTOR_REV       (13.0f)
#define ENCODER_GEAR_RATIO                (30.0f)
#define ENCODER_DECODE_MULTIPLIER         (4.0f)
#define ENCODER_COUNTS_PER_WHEEL_REV      \
    (ENCODER_LINES_PER_MOTOR_REV * ENCODER_GEAR_RATIO * \
     ENCODER_DECODE_MULTIPLIER)
#define ENCODER_WHEEL_DIAMETER_MM         (65.0f)
#define ENCODER_WHEEL_CIRCUMFERENCE_MM    (204.2035f)
#define ENCODER_COUNTS_PER_MM             \
    (ENCODER_COUNTS_PER_WHEEL_REV / ENCODER_WHEEL_CIRCUMFERENCE_MM)

typedef struct {
    int32_t signed_counts_a;
    int32_t signed_counts_b;
    uint32_t travel_counts_a;
    uint32_t travel_counts_b;
    float distance_a_mm;
    float distance_b_mm;
    float distance_center_mm;
} encoder_odometry_t;

typedef struct {
    uint8_t previous_state;
} encoder_quadrature_t;

void Encoder_QuadratureInit(encoder_quadrature_t *decoder,
                            GPIO_Regs *port,
                            uint32_t phase_a_pin,
                            uint32_t phase_b_pin);
void Encoder_QuadratureUpdate(encoder_quadrature_t *decoder,
                              GPIO_Regs *port,
                              uint32_t phase_a_pin,
                              uint32_t phase_b_pin,
                              volatile int *encoder_count);
int ReadSpeed(volatile int *EncoderCount);
void Encoder_OdometryReset(void);
void Encoder_OdometryUpdate(int16_t delta_a, int16_t delta_b);
void Encoder_GetOdometry(encoder_odometry_t *odometry);
float Encoder_CountsToMm(float counts);
float Encoder_GetDistanceCenterMm(void);

#endif
