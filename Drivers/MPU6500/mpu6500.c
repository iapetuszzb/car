#include "mpu6500.h"

#include "clock.h"
#include "ti_msp_dl_config.h"

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
#define MPU6500_CALIB_SAMPLES    (500U)
#define MPU6500_CALIB_MIN_SAMPLES (100U)
#define MPU6500_CALIB_DELAY_MS   (10U)
#define MPU6500_CALIB_SETTLE_MS  (1000U)

#define GYRO_Z_DEADBAND_DPS          (0.0f)

#define MPU6500_I2C_INST I2C_MPU6500_INST

short gyro[3], accel[3];
float pitch, roll, yaw;

static uint8_t mpu6500_addr = MPU6500_ADDR_LOW;
static float gyro_bias[3];
static unsigned long mpu6500_last_ms;

static uint8_t mpu6500_write(uint8_t reg, const uint8_t *buf, uint8_t len);
static uint8_t mpu6500_read(uint8_t reg, uint8_t *buf, uint8_t len);
static uint8_t mpu6500_write_byte(uint8_t reg, uint8_t data);
static uint8_t mpu6500_detect_address(void);
static uint8_t mpu6500_read_raw(void);
static uint8_t mpu6500_calibrate_gyro(void);
static int16_t mpu6500_to_int16(uint8_t high, uint8_t low);
static float mpu6500_absf(float x);
static float mpu6500_wrap_angle_deg(float angle);

uint8_t MPU6500_Init(void)
{
    mspm0_delay_ms(100);

    if (mpu6500_detect_address() != 0) {
        return 2;
    }

    if (mpu6500_write_byte(MPU6500_REG_PWR_MGMT_1, 0x80) != 0) {
        return 1;
    }
    mspm0_delay_ms(100);

    if (mpu6500_write_byte(MPU6500_REG_PWR_MGMT_1, 0x00) != 0) {
        return 1;
    }
    mspm0_delay_ms(100);

    if (MPU6500_ReadID() != MPU6500_WHO_AM_I_VALUE) {
        return 2;
    }

    (void)mpu6500_write_byte(MPU6500_REG_PWR_MGMT_2, 0x00);
    (void)mpu6500_write_byte(MPU6500_REG_SMPLRT_DIV, 0x09);
    if (mpu6500_write_byte(MPU6500_REG_CONFIG, 0x03) != 0) {
        return 1;
    }
    if (mpu6500_write_byte(MPU6500_REG_GYRO_CONFIG, 0x00) != 0) {
        return 1;
    }
    if (mpu6500_write_byte(MPU6500_REG_ACCEL_CONFIG, 0x00) != 0) {
        return 1;
    }
    (void)mpu6500_write_byte(MPU6500_REG_ACCEL_CONFIG2, 0x03);

    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    mpu6500_last_ms = 0;

    if (mpu6500_calibrate_gyro() != 0) {
        return 3;
    }

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
    unsigned long now_ms;

    if (mpu6500_read_raw() != 0) {
        return 1;
    }

    if (mspm0_get_clock_ms(&now_ms) == 0) {
        if (mpu6500_last_ms != 0) {
            float dt_s = (float)(now_ms - mpu6500_last_ms) / 1000.0f;
            float gz_dps = (float)gyro[2] / MPU6500_GYRO_LSB_PER_DPS;
            float gz_corr_raw = gz_dps - gyro_bias[2];
            float gz_corr_db;

            if (dt_s <= 0.0f || dt_s > 0.5f) {
                dt_s = 0.1f;
            }

            if (mpu6500_absf(gz_corr_raw) < GYRO_Z_DEADBAND_DPS) {
                gz_corr_db = 0.0f;
            } else {
                gz_corr_db = gz_corr_raw;
            }

            yaw = mpu6500_wrap_angle_deg(yaw + gz_corr_db * dt_s);
        }
        mpu6500_last_ms = now_ms;
    }

    return 0;
}

static uint8_t mpu6500_write(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t cnt = len;
    const uint8_t *ptr = buf;
    unsigned long start, cur;

    if (len == 0) {
        return 0;
    }

    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(MPU6500_I2C_INST, reg);
    DL_I2C_clearInterruptStatus(MPU6500_I2C_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    while (!(DL_I2C_getControllerStatus(MPU6500_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
    }

    DL_I2C_startControllerTransfer(MPU6500_I2C_INST, mpu6500_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);

    do {
        uint8_t fillcnt;
        fillcnt = DL_I2C_fillControllerTXFIFO(MPU6500_I2C_INST, ptr, cnt);
        cnt -= fillcnt;
        ptr += fillcnt;

        mspm0_get_clock_ms(&cur);
        if (cur >= (start + MPU6500_I2C_TIMEOUT_MS)) {
            return 1;
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

    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(MPU6500_I2C_INST, reg);
    MPU6500_I2C_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(MPU6500_I2C_INST,
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    while (!(DL_I2C_getControllerStatus(MPU6500_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
    }

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
        if (cur >= (start + MPU6500_I2C_TIMEOUT_MS)) {
            MPU6500_I2C_INST->MASTER.MCTR = 0;
            return 1;
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
    uint8_t data[14];

    if (mpu6500_read(MPU6500_REG_ACCEL_XOUT_H, data, sizeof(data)) != 0) {
        return 1;
    }

    accel[0] = mpu6500_to_int16(data[0], data[1]);
    accel[1] = mpu6500_to_int16(data[2], data[3]);
    accel[2] = mpu6500_to_int16(data[4], data[5]);
    gyro[0] = mpu6500_to_int16(data[8], data[9]);
    gyro[1] = mpu6500_to_int16(data[10], data[11]);
    gyro[2] = mpu6500_to_int16(data[12], data[13]);

    return 0;
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

    return (sample_count >= MPU6500_CALIB_MIN_SAMPLES) ? 0 : 1;
}

static int16_t mpu6500_to_int16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static float mpu6500_absf(float x)
{
    return (x < 0.0f) ? -x : x;
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
