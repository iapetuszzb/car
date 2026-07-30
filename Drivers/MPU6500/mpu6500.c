#include "mpu6500.h"

#include "clock.h"
#include "ti_msp_dl_config.h"

#include <math.h>

#define MPU6500_ADDR_LOW         (0x68)
#define MPU6500_ADDR_HIGH        (0x69)
#define MPU6500_WHO_AM_I_VALUE   (0x70)

#define MPU6500_REG_SMPLRT_DIV   (0x19)
#define MPU6500_REG_CONFIG       (0x1A)
#define MPU6500_REG_GYRO_CONFIG  (0x1B)
#define MPU6500_REG_ACCEL_CONFIG (0x1C)
#define MPU6500_REG_ACCEL_CONFIG2 (0x1D)
#define MPU6500_REG_ACCEL_XOUT_H (0x3B)
#define MPU6500_REG_PWR_MGMT_1   (0x6B)
#define MPU6500_REG_PWR_MGMT_2   (0x6C)
#define MPU6500_REG_WHO_AM_I     (0x75)

#define MPU6500_I2C_TIMEOUT_MS   (10)
#define MPU6500_GYRO_LSB_PER_DPS (131.0f)
#define MPU6500_ACCEL_LSB_PER_G  (16384.0f)
#define MPU6500_RAW_FRAME_LEN    (14U)
#define MPU6500_CALIB_SAMPLES    (500U)
#define MPU6500_CALIB_MIN_SAMPLES (100U)
#define MPU6500_CALIB_DELAY_MS   (10U)
#define MPU6500_CALIB_SETTLE_MS  (1000U)

#define MPU6500_YAW_RATE_DEADBAND_DPS   (1.5f)
#define MPU6500_STILL_BIAS_ADAPT_ENABLE (1)
#define MPU6500_STILL_GZ_THRESH_DPS     (0.20f)
#define MPU6500_STILL_GXY_THRESH_DPS    (1.50f)
#define MPU6500_STILL_ACC_NORM_MIN_G    (0.85f)
#define MPU6500_STILL_ACC_NORM_MAX_G    (1.15f)
#define MPU6500_STILL_BIAS_TAU_S        (8.0f)
#define MPU6500_STILL_YAW_HOLD_DPS      (0.02f)
#define MPU6500_MAHONY_KP               (2.0f)
#define MPU6500_MAHONY_KI               (0.04f)
#define MPU6500_DEG_TO_RAD              (0.01745329252f)
#define MPU6500_RAD_TO_DEG              (57.2957795131f)

#define MPU6500_I2C_INST I2C_MPU6500_INST

short gyro[3], accel[3];
float pitch, roll, yaw;

static uint8_t mpu6500_addr = MPU6500_ADDR_LOW;
static volatile uint8_t mpu6500_read_busy;
static float gyro_bias[3];
static float mpu6500_yaw_rate_dps;
static unsigned long mpu6500_last_ms;
static volatile uint8_t mpu6500_init_status = 0xffU;
static volatile uint8_t mpu6500_last_read_status = 2U;
static volatile uint32_t mpu6500_update_count;
static float mahony_q[4];
static float mahony_integral[3];

static uint8_t mpu6500_write(uint8_t reg, const uint8_t *buf, uint8_t len);
static uint8_t mpu6500_read(uint8_t reg, uint8_t *buf, uint8_t len);
static uint8_t mpu6500_wait_controller_idle(void);
static void mpu6500_abort_transfer(void);
static uint8_t mpu6500_write_byte(uint8_t reg, uint8_t data);
static uint8_t mpu6500_detect_address(void);
static uint8_t mpu6500_read_raw(void);
static void mpu6500_parse_raw_frame(const uint8_t *data);
static uint8_t mpu6500_calibrate_gyro(void);
static void mpu6500_update_attitude(unsigned long now_ms);
static void mpu6500_mahony_reset(void);
static void mpu6500_mahony_update(float gx_dps, float gy_dps, float gz_dps,
                                  float ax_g, float ay_g, float az_g,
                                  float dt_s);
static void mpu6500_update_euler_from_quat(void);
static int16_t mpu6500_to_int16(uint8_t high, uint8_t low);
static float mpu6500_absf(float x);
static float mpu6500_apply_deadband(float value, float deadband);
static float mpu6500_wrap_angle_deg(float angle);

uint8_t MPU6500_Init(void)
{
    mpu6500_init_status = 0xfeU;
    mpu6500_last_read_status = 2U;
    mpu6500_update_count = 0U;

    mspm0_delay_ms(100);

    if (mpu6500_detect_address() != 0) {
        mpu6500_init_status = 2U;
        return 2;
    }

    if (mpu6500_write_byte(MPU6500_REG_PWR_MGMT_1, 0x80) != 0) {
        mpu6500_init_status = 1U;
        return 1;
    }
    mspm0_delay_ms(100);

    if (mpu6500_write_byte(MPU6500_REG_PWR_MGMT_1, 0x00) != 0) {
        mpu6500_init_status = 1U;
        return 1;
    }
    mspm0_delay_ms(100);

    if (MPU6500_ReadID() != MPU6500_WHO_AM_I_VALUE) {
        mpu6500_init_status = 2U;
        return 2;
    }

    (void)mpu6500_write_byte(MPU6500_REG_PWR_MGMT_2, 0x00);
    (void)mpu6500_write_byte(MPU6500_REG_SMPLRT_DIV, 0x09);
    if (mpu6500_write_byte(MPU6500_REG_CONFIG, 0x03) != 0) {
        mpu6500_init_status = 1U;
        return 1;
    }
    if (mpu6500_write_byte(MPU6500_REG_GYRO_CONFIG, 0x00) != 0) {
        mpu6500_init_status = 1U;
        return 1;
    }
    if (mpu6500_write_byte(MPU6500_REG_ACCEL_CONFIG, 0x00) != 0) {
        mpu6500_init_status = 1U;
        return 1;
    }
    (void)mpu6500_write_byte(MPU6500_REG_ACCEL_CONFIG2, 0x03);

    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    mpu6500_yaw_rate_dps = 0.0f;
    mpu6500_last_ms = 0;
    mpu6500_mahony_reset();

    if (mpu6500_calibrate_gyro() != 0) {
        mpu6500_init_status = 3U;
        return 3;
    }

    mpu6500_init_status = 0U;
    return 0;
}

uint8_t MPU6500_ReadID(void)
{
    uint8_t id = 0xff;
    (void)mpu6500_read(MPU6500_REG_WHO_AM_I, &id, 1);
    return id;
}

uint8_t Read_MPU6500(void)
{
    unsigned long now_ms = 0;

    if (mpu6500_init_status != 0U) {
        mpu6500_last_read_status = 1U;
        return 1U;
    }

    if (mpu6500_read_busy != 0U) {
        mpu6500_last_read_status = 2U;
        return 2;
    }
    mpu6500_read_busy = 1U;

    if (mpu6500_read_raw() != 0U) {
        mpu6500_read_busy = 0U;
        mpu6500_last_read_status = 1U;
        return 1U;
    }

    (void)mspm0_get_clock_ms(&now_ms);
    mpu6500_update_attitude(now_ms);

    mpu6500_read_busy = 0U;
    mpu6500_last_read_status = 0U;
    return 0U;
}

float MPU6500_GetYawRateDps(void)
{
    return mpu6500_yaw_rate_dps;
}

uint8_t MPU6500_GetInitStatus(void)
{
    return mpu6500_init_status;
}

uint8_t MPU6500_GetLastReadStatus(void)
{
    return mpu6500_last_read_status;
}

uint32_t MPU6500_GetUpdateCount(void)
{
    return mpu6500_update_count;
}

static uint8_t mpu6500_write(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t cnt = len;
    const uint8_t *ptr = buf;
    unsigned long start, cur;

    if (len == 0) {
        return 0;
    }

    if (mpu6500_wait_controller_idle() != 0U) {
        return 1U;
    }

    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(MPU6500_I2C_INST, reg);
    DL_I2C_clearInterruptStatus(MPU6500_I2C_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    DL_I2C_startControllerTransfer(MPU6500_I2C_INST, mpu6500_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);

    do {
        uint8_t fillcnt;
        fillcnt = DL_I2C_fillControllerTXFIFO(MPU6500_I2C_INST, ptr, cnt);
        cnt -= fillcnt;
        ptr += fillcnt;

        mspm0_get_clock_ms(&cur);
        if ((unsigned long)(cur - start) >= MPU6500_I2C_TIMEOUT_MS) {
            mpu6500_abort_transfer();
            return 1U;
        }
    } while (!DL_I2C_getRawInterruptStatus(MPU6500_I2C_INST,
                                           DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

    return 0;
}

static uint8_t mpu6500_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;
    unsigned long start, cur;

    if (len == 0) {
        return 0;
    }

    if (mpu6500_wait_controller_idle() != 0U) {
        return 1U;
    }

    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(MPU6500_I2C_INST, reg);
    MPU6500_I2C_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(MPU6500_I2C_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    DL_I2C_startControllerTransfer(MPU6500_I2C_INST, mpu6500_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    do {
        if (!DL_I2C_isControllerRXFIFOEmpty(MPU6500_I2C_INST)) {
            if (i < len) {
                buf[i++] = DL_I2C_receiveControllerData(MPU6500_I2C_INST);
            } else {
                (void)DL_I2C_receiveControllerData(MPU6500_I2C_INST);
            }
        }

        mspm0_get_clock_ms(&cur);
        if ((unsigned long)(cur - start) >= MPU6500_I2C_TIMEOUT_MS) {
            mpu6500_abort_transfer();
            return 1U;
        }
    } while (!DL_I2C_getRawInterruptStatus(MPU6500_I2C_INST,
                                           DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    while (!DL_I2C_isControllerRXFIFOEmpty(MPU6500_I2C_INST) && i < len) {
        buf[i++] = DL_I2C_receiveControllerData(MPU6500_I2C_INST);
    }

    MPU6500_I2C_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(MPU6500_I2C_INST);

    return (i == len) ? 0 : 1;
}

static uint8_t mpu6500_wait_controller_idle(void)
{
    unsigned long start;
    unsigned long now;

    (void)mspm0_get_clock_ms(&start);
    while (!(DL_I2C_getControllerStatus(MPU6500_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        (void)mspm0_get_clock_ms(&now);
        if ((unsigned long)(now - start) >= MPU6500_I2C_TIMEOUT_MS) {
            mpu6500_abort_transfer();
            return 1U;
        }
    }

    return 0U;
}

static void mpu6500_abort_transfer(void)
{
    DL_I2C_resetControllerTransfer(MPU6500_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU6500_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU6500_I2C_INST);
    DL_I2C_clearInterruptStatus(
        MPU6500_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
}

static uint8_t mpu6500_write_byte(uint8_t reg, uint8_t data)
{
    return mpu6500_write(reg, &data, 1);
}

static uint8_t mpu6500_detect_address(void)
{
    mpu6500_addr = MPU6500_ADDR_LOW;
    if (MPU6500_ReadID() == MPU6500_WHO_AM_I_VALUE) {
        return 0;
    }

    mpu6500_addr = MPU6500_ADDR_HIGH;
    if (MPU6500_ReadID() == MPU6500_WHO_AM_I_VALUE) {
        return 0;
    }

    mpu6500_addr = MPU6500_ADDR_LOW;
    return 1;
}

static uint8_t mpu6500_read_raw(void)
{
    uint8_t data[MPU6500_RAW_FRAME_LEN];

    if (mpu6500_read(MPU6500_REG_ACCEL_XOUT_H, data, sizeof(data)) != 0) {
        return 1;
    }

    mpu6500_parse_raw_frame(data);

    return 0;
}

static void mpu6500_parse_raw_frame(const uint8_t *data)
{
    accel[0] = mpu6500_to_int16(data[0], data[1]);
    accel[1] = mpu6500_to_int16(data[2], data[3]);
    accel[2] = mpu6500_to_int16(data[4], data[5]);
    gyro[0] = mpu6500_to_int16(data[8], data[9]);
    gyro[1] = mpu6500_to_int16(data[10], data[11]);
    gyro[2] = mpu6500_to_int16(data[12], data[13]);
}

static uint8_t mpu6500_calibrate_gyro(void)
{
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t sample_count = 0;
    uint16_t attempts = 0;

    mspm0_delay_ms(MPU6500_CALIB_SETTLE_MS);

    while (sample_count < MPU6500_CALIB_SAMPLES && attempts < (MPU6500_CALIB_SAMPLES * 2U)) {
        attempts++;
        if (mpu6500_read_raw() == 0) {
            gyro_sum[0] += (float)gyro[0] / MPU6500_GYRO_LSB_PER_DPS;
            gyro_sum[1] += (float)gyro[1] / MPU6500_GYRO_LSB_PER_DPS;
            gyro_sum[2] += (float)gyro[2] / MPU6500_GYRO_LSB_PER_DPS;
            sample_count++;
        }
        mspm0_delay_ms(MPU6500_CALIB_DELAY_MS);
    }

    if (sample_count == 0) {
        return 1;
    }

    gyro_bias[0] = gyro_sum[0] / (float)sample_count;
    gyro_bias[1] = gyro_sum[1] / (float)sample_count;
    gyro_bias[2] = gyro_sum[2] / (float)sample_count;
    mpu6500_last_ms = 0;
    yaw = 0.0f;
    mpu6500_yaw_rate_dps = 0.0f;
    mpu6500_mahony_reset();

    return (sample_count >= MPU6500_CALIB_MIN_SAMPLES) ? 0 : 1;
}

static void mpu6500_update_attitude(unsigned long now_ms)
{
    float dt_s;
    float ax_g;
    float ay_g;
    float az_g;
    float acc_norm;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float gz_corr_raw;
    float gx_corr;
    float gy_corr;
    float gz_corr;
    uint8_t still_for_bias = 0U;

    if (mpu6500_last_ms == 0U) {
        mpu6500_last_ms = now_ms;
        mpu6500_update_count++;
        return;
    }

    dt_s = (float)(now_ms - mpu6500_last_ms) / 1000.0f;
    if (dt_s <= 0.0f) {
        return;
    }
    if (dt_s > 0.25f) {
        dt_s = 0.25f;
    }
    mpu6500_last_ms = now_ms;

    ax_g = (float)accel[0] / MPU6500_ACCEL_LSB_PER_G;
    ay_g = (float)accel[1] / MPU6500_ACCEL_LSB_PER_G;
    az_g = (float)accel[2] / MPU6500_ACCEL_LSB_PER_G;
    acc_norm = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
    gx_dps = (float)gyro[0] / MPU6500_GYRO_LSB_PER_DPS;
    gy_dps = (float)gyro[1] / MPU6500_GYRO_LSB_PER_DPS;
    gz_dps = (float)gyro[2] / MPU6500_GYRO_LSB_PER_DPS;
    gz_corr_raw = gz_dps - gyro_bias[2];
    gx_corr = gx_dps - gyro_bias[0];
    gy_corr = gy_dps - gyro_bias[1];
    gz_corr = gz_corr_raw;

#if MPU6500_STILL_BIAS_ADAPT_ENABLE
    if ((mpu6500_absf(gz_corr_raw) < MPU6500_STILL_GZ_THRESH_DPS) &&
        (mpu6500_absf(gx_dps - gyro_bias[0]) < MPU6500_STILL_GXY_THRESH_DPS) &&
        (mpu6500_absf(gy_dps - gyro_bias[1]) < MPU6500_STILL_GXY_THRESH_DPS) &&
        (acc_norm >= MPU6500_STILL_ACC_NORM_MIN_G) &&
        (acc_norm <= MPU6500_STILL_ACC_NORM_MAX_G)) {
        float alpha = dt_s / MPU6500_STILL_BIAS_TAU_S;
        if (alpha > 0.05f) {
            alpha = 0.05f;
        }
        if (alpha > 0.0f) {
            gyro_bias[2] += alpha * gz_corr_raw;
            gz_corr_raw = gz_dps - gyro_bias[2];
            gz_corr = gz_corr_raw;
        }
        still_for_bias = 1U;
    }
#endif

    /* Suppress low-rate Z-axis drift before yaw fusion and angle PID use. */
    gz_corr = mpu6500_apply_deadband(
        gz_corr, MPU6500_YAW_RATE_DEADBAND_DPS);

#if MPU6500_STILL_BIAS_ADAPT_ENABLE
    if ((still_for_bias != 0U) &&
        (mpu6500_absf(gz_corr) < MPU6500_STILL_YAW_HOLD_DPS)) {
        gz_corr = 0.0f;
    }
#endif

    mpu6500_yaw_rate_dps = gz_corr;

    /* A 6-axis IMU has no absolute yaw reference. Integrate the calibrated
     * Z gyro directly for chassis heading; accelerometer fusion is retained
     * below for roll and pitch only. */
    yaw = mpu6500_wrap_angle_deg(yaw + (gz_corr * dt_s));

    mpu6500_mahony_update(gx_corr, gy_corr, gz_corr,
                          ax_g, ay_g, az_g, dt_s);
    mpu6500_update_euler_from_quat();
    mpu6500_update_count++;
}

static void mpu6500_mahony_reset(void)
{
    mahony_q[0] = 1.0f;
    mahony_q[1] = 0.0f;
    mahony_q[2] = 0.0f;
    mahony_q[3] = 0.0f;
    mahony_integral[0] = 0.0f;
    mahony_integral[1] = 0.0f;
    mahony_integral[2] = 0.0f;
}

static void mpu6500_mahony_update(float gx_dps, float gy_dps, float gz_dps,
                                  float ax_g, float ay_g, float az_g,
                                  float dt_s)
{
    float q0 = mahony_q[0];
    float q1 = mahony_q[1];
    float q2 = mahony_q[2];
    float q3 = mahony_q[3];
    float acc_norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float gx;
    float gy;
    float gz;
    float q_dot0;
    float q_dot1;
    float q_dot2;
    float q_dot3;
    float q_norm;

    if (dt_s <= 0.0f) {
        dt_s = 0.01f;
    }

    acc_norm = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
    if (acc_norm > 0.001f) {
        ax_g /= acc_norm;
        ay_g /= acc_norm;
        az_g /= acc_norm;

        vx = 2.0f * ((q1 * q3) - (q0 * q2));
        vy = 2.0f * ((q0 * q1) + (q2 * q3));
        vz = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);

        ex = (ay_g * vz) - (az_g * vy);
        ey = (az_g * vx) - (ax_g * vz);
        ez = (ax_g * vy) - (ay_g * vx);

        mahony_integral[0] += MPU6500_MAHONY_KI * ex * dt_s;
        mahony_integral[1] += MPU6500_MAHONY_KI * ey * dt_s;
        mahony_integral[2] += MPU6500_MAHONY_KI * ez * dt_s;

        gx_dps += (MPU6500_MAHONY_KP * ex + mahony_integral[0]) *
                  MPU6500_RAD_TO_DEG;
        gy_dps += (MPU6500_MAHONY_KP * ey + mahony_integral[1]) *
                  MPU6500_RAD_TO_DEG;
        gz_dps += (MPU6500_MAHONY_KP * ez + mahony_integral[2]) *
                  MPU6500_RAD_TO_DEG;
    }

    gx = gx_dps * MPU6500_DEG_TO_RAD;
    gy = gy_dps * MPU6500_DEG_TO_RAD;
    gz = gz_dps * MPU6500_DEG_TO_RAD;

    q_dot0 = -0.5f * ((q1 * gx) + (q2 * gy) + (q3 * gz));
    q_dot1 =  0.5f * ((q0 * gx) + (q2 * gz) - (q3 * gy));
    q_dot2 =  0.5f * ((q0 * gy) - (q1 * gz) + (q3 * gx));
    q_dot3 =  0.5f * ((q0 * gz) + (q1 * gy) - (q2 * gx));

    q0 += q_dot0 * dt_s;
    q1 += q_dot1 * dt_s;
    q2 += q_dot2 * dt_s;
    q3 += q_dot3 * dt_s;

    q_norm = sqrtf((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
    if (q_norm > 0.001f) {
        mahony_q[0] = q0 / q_norm;
        mahony_q[1] = q1 / q_norm;
        mahony_q[2] = q2 / q_norm;
        mahony_q[3] = q3 / q_norm;
    } else {
        mpu6500_mahony_reset();
    }
}

static void mpu6500_update_euler_from_quat(void)
{
    float q0 = mahony_q[0];
    float q1 = mahony_q[1];
    float q2 = mahony_q[2];
    float q3 = mahony_q[3];
    float sinp;

    roll = atan2f(2.0f * ((q0 * q1) + (q2 * q3)),
                  1.0f - (2.0f * ((q1 * q1) + (q2 * q2)))) *
           MPU6500_RAD_TO_DEG;

    sinp = 2.0f * ((q0 * q2) - (q3 * q1));
    if (sinp >= 1.0f) {
        pitch = 90.0f;
    } else if (sinp <= -1.0f) {
        pitch = -90.0f;
    } else {
        pitch = asinf(sinp) * MPU6500_RAD_TO_DEG;
    }

    /* Yaw is maintained by direct Z-gyro integration in
     * mpu6500_update_attitude(). */
}

static int16_t mpu6500_to_int16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static float mpu6500_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float mpu6500_apply_deadband(float value, float deadband)
{
    return (mpu6500_absf(value) <= deadband) ? 0.0f : value;
}

static float mpu6500_wrap_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}
