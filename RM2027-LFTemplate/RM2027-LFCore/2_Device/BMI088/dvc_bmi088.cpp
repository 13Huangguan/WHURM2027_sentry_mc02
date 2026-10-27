#include "dvc_bmi088.hpp"

#include "cmsis_os.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include <cmath>

enum class Bmi088Sensor { ACCEL, GYRO };

static void selectSensor(Bmi088Sensor sensor, bool selected) {
    if (sensor == Bmi088Sensor::ACCEL) {
        if (selected) {
            bmi088AccelNsL();
        } else {
            bmi088AccelNsH();
        }
    } else {
        if (selected) {
            bmi088GyroNsL();
        } else {
            bmi088GyroNsH();
        }
    }
}

static void writeRegister(Bmi088Sensor sensor, uint8_t reg, uint8_t data) {
    selectSensor(sensor, true);
    bmi088ReadWriteByte(reg);
    bmi088ReadWriteByte(data);
    selectSensor(sensor, false);
}

static void readRegisters(Bmi088Sensor sensor, uint8_t reg, uint8_t* data, uint8_t count) {
    selectSensor(sensor, true);
    bmi088ReadWriteByte(reg | 0x80);
    if (sensor == Bmi088Sensor::ACCEL) {
        bmi088ReadWriteByte(0x55);
    }
    for (uint8_t index = 0; index < count; ++index) {
        data[index] = bmi088ReadWriteByte(0x55);
    }
    selectSensor(sensor, false);
}

static uint8_t readRegister(Bmi088Sensor sensor, uint8_t reg) {
    uint8_t data;
    readRegisters(sensor, reg, &data, 1);
    return data;
}

static constexpr float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;

static constexpr float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

Bmi088Info bmi088_info;

static constexpr uint8_t ACCEL_REGISTER_CONFIGURATION[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    /* Turn on accelerometer */
    { BMI088_ACCEL_PWR_CTRL, BMI088_ACCEL_ENABLE_ACCEL_ON, BMI088_ACCEL_PWR_CTRL_ERROR },

    /* Pause mode */
    { BMI088_ACCEL_PWR_CONF, BMI088_ACCEL_PWR_ACTIVE_MODE, BMI088_ACCEL_PWR_CONF_ERROR },

    /* ACCELeration Configuration */
    { BMI088_ACCEL_CONF,
      (BMI088_ACCEL_NORMAL | BMI088_ACCEL_1600_HZ | BMI088_ACCEL_CONF_MUST_SET),
      BMI088_ACCEL_CONF_ERROR },

    /* ACCELerometer setting range */
    { BMI088_ACCEL_RANGE, BMI088_ACCEL_RANGE_6G, BMI088_ACCEL_RANGE_ERROR },

    /* INT1 Configuration input and output pin */
    { BMI088_INT1_IO_CTRL,
      (BMI088_ACCEL_INT1_IO_ENABLE | BMI088_ACCEL_INT1_GPIO_PP | BMI088_ACCEL_INT1_GPIO_LOW),
      BMI088_INT1_IO_CTRL_ERROR },

    /* interrupt map pin */
    { BMI088_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR }
};

static constexpr uint8_t GYRO_REGISTER_CONFIGURATION[BMI088_WRITE_GYRO_REG_NUM][3] = {
    /* Angular rate and resolution */
    { BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR },

    /* Data Transfer Rate and Bandwidth Settings */
    { BMI088_GYRO_BANDWIDTH,
      (BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET),
      BMI088_GYRO_BANDWIDTH_ERROR },

    /* Power Mode Selection Register */
    { BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR },

    /* Data Interrupt Trigger Register */
    { BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR },

    /* Interrupt Pin Trigger Register */
    { BMI088_GYRO_INT3_INT4_IO_CONF,
      (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW),
      BMI088_GYRO_INT3_INT4_IO_CONF_ERROR },

    /* interrupt map register */
    { BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR }
};

static Bmi088Status bmi088AccelInit(void) {
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the accelerator ID address */
    res = readRegister(Bmi088Sensor::ACCEL, BMI088_ACCEL_CHIP_ID);
    /* waiting 150us - replaced with osDelay for RTOS compatibility */
    osDelay(1);
    /* read again */
    res = readRegister(Bmi088Sensor::ACCEL, BMI088_ACCEL_CHIP_ID);
    /* waiting 150us - replaced with osDelay for RTOS compatibility */
    osDelay(1);

    /* ACCELerator software Reset ------------------------------------------------*/
    /* write 0xB6 to the register ACCEL_SOFTResET that addRess is 0x7E to reset software */
    writeRegister(Bmi088Sensor::ACCEL, BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE);
    /* software reset waiting time: 80ms - using osDelay instead of Delay_ms */
    osDelay(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    res = readRegister(Bmi088Sensor::ACCEL, BMI088_ACCEL_CHIP_ID);
    osDelay(1);
    res = readRegister(Bmi088Sensor::ACCEL, BMI088_ACCEL_CHIP_ID);
    osDelay(1);

    /* if the value of the 0x00 register is not 0x1E,reset failed */
    if (res != BMI088_ACCEL_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    /* config the accelerator sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        writeRegister(
            Bmi088Sensor::ACCEL,
            ACCEL_REGISTER_CONFIGURATION[write_reg_num][0],
            ACCEL_REGISTER_CONFIGURATION[write_reg_num][1]
        );
        /* waiting 150us - replaced with osDelay for RTOS compatibility */
        osDelay(1);

        /* read the configuration */
        res = readRegister(Bmi088Sensor::ACCEL, ACCEL_REGISTER_CONFIGURATION[write_reg_num][0]);
        /* waiting 150us - replaced with osDelay for RTOS compatibility */
        osDelay(1);

        /* check the configuration and return the specified error */
        if (res != ACCEL_REGISTER_CONFIGURATION[write_reg_num][1]) {
            return (Bmi088Status)ACCEL_REGISTER_CONFIGURATION[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;
}

static Bmi088Status bmi088GyroInit(void) {
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the gyro ID address */
    res = readRegister(Bmi088Sensor::GYRO, BMI088_GYRO_CHIP_ID);
    /* waiting 150us - replaced with osDelay for RTOS compatibility */
    osDelay(1);
    /* read again */
    res = readRegister(Bmi088Sensor::GYRO, BMI088_GYRO_CHIP_ID);
    /* waiting 150us - replaced with osDelay for RTOS compatibility */
    osDelay(1);

    /* gyro software reset ------------------------------------------------*/
    /* write 0xB6 to the register GYRO_SOFTRESET that address is 0x14 to reset software */
    writeRegister(Bmi088Sensor::GYRO, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    /* software reset waiting time: 80ms - using osDelay instead of Delay_ms */
    osDelay(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    res = readRegister(Bmi088Sensor::GYRO, BMI088_GYRO_CHIP_ID);
    osDelay(1);
    res = readRegister(Bmi088Sensor::GYRO, BMI088_GYRO_CHIP_ID);
    osDelay(1);

    /* if the value of the 0x00 register is not 0x0F,reset failed */
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    /* config the gyro sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        writeRegister(
            Bmi088Sensor::GYRO,
            GYRO_REGISTER_CONFIGURATION[write_reg_num][0],
            GYRO_REGISTER_CONFIGURATION[write_reg_num][1]
        );
        /* waiting 150us - replaced with osDelay for RTOS compatibility */
        osDelay(1);

        /* read the configuration */
        res = readRegister(Bmi088Sensor::GYRO, GYRO_REGISTER_CONFIGURATION[write_reg_num][0]);
        /* waiting 150us - replaced with osDelay for RTOS compatibility */
        osDelay(1);

        /* check the configuration and return the specified error */
        if (res != GYRO_REGISTER_CONFIGURATION[write_reg_num][1]) {
            return (Bmi088Status)GYRO_REGISTER_CONFIGURATION[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;
}

/* 与 ins_config.hpp 保持一致：关闭开机 5s 静态标定，使用实测零偏。
   注意 dvc_bmi088.cpp 不包含 ins_config.hpp，宏必须在这里给出默认值。 */
#ifndef IMU_CALIBRATION_ENABLE
    #define IMU_CALIBRATION_ENABLE 0U
#endif

static void bmi088OffsetUpdate(Bmi088Info* info) {
#if IMU_CALIBRATION_ENABLE /* ENABLE the BMI088 Calibration */

    uint8_t buf[8] = {
        0,
    };

    info->offsets_gyro_x = info->offsets_gyro_y = info->offsets_gyro_z = 0.0f;
    uint16_t valid_samples = 0;
    for (uint16_t i = 0; i < 5000; i++) {
        /* read the accelerator multi data */
        readRegisters(Bmi088Sensor::ACCEL, BMI088_ACCEL_XOUT_L, buf, 6);
        info->mpu_info.accel_x = (int16_t)((buf[1]) << 8) | buf[0];
        info->mpu_info.accel_y = (int16_t)((buf[3]) << 8) | buf[2];
        info->mpu_info.accel_z = (int16_t)((buf[5]) << 8) | buf[4];

        /* read the gyro multi data */
        readRegisters(Bmi088Sensor::GYRO, BMI088_GYRO_CHIP_ID, buf, 8);
        /* check the ID */
        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
            info->mpu_info.gyro_x = (int16_t)((buf[3]) << 8) | buf[2];
            info->mpu_info.gyro_y = (int16_t)((buf[5]) << 8) | buf[4];
            info->mpu_info.gyro_z = (int16_t)((buf[7]) << 8) | buf[6];

            ++valid_samples;
            /* update the gyro offsets */
            info->offsets_gyro_x += BMI088_GYRO_SEN * info->mpu_info.gyro_x;
            info->offsets_gyro_y += BMI088_GYRO_SEN * info->mpu_info.gyro_y;
            info->offsets_gyro_z += BMI088_GYRO_SEN * info->mpu_info.gyro_z;
        }
        /* waiting 1ms - using osDelay instead of Delay_ms for RTOS safety */
        osDelay(1);
    }

    info->offsets_gyro_x = valid_samples > 0 ? info->offsets_gyro_x / valid_samples : 0.0f;
    info->offsets_gyro_y = valid_samples > 0 ? info->offsets_gyro_y / valid_samples : 0.0f;
    info->offsets_gyro_z = valid_samples > 0 ? info->offsets_gyro_z / valid_samples : 0.0f;

#else /* DISABLE the BMI088 Calibration */
    /* 融合前实测的零偏，改板子或换 IMU 后必须重新标定 */
    info->offsets_gyro_x = 0.00405759178f;
    info->offsets_gyro_y = 0.000725232065f;
    info->offsets_gyro_z = 0.00128263142f;
#endif

    /* set the offset init flag */
    info->offsets_init = true;
}

void bmi088Init(void) {
    uint8_t status = BMI088_NO_ERROR;

    /* Initializes the BMI088 */
    do {
        status = bmi088AccelInit();

        status |= bmi088GyroInit();

        /* using osDelay instead of Delay_ms for RTOS safety */
        osDelay(2);

    } while (status);

    bmi088OffsetUpdate(&bmi088_info);
}

void bmi088InfoUpdate(Bmi088Info* info) {
    uint8_t buf[8] = { 0, 0, 0, 0, 0, 0 };

    /* read the accelerator multi data */
    readRegisters(Bmi088Sensor::ACCEL, BMI088_ACCEL_XOUT_L, buf, 6);
    info->mpu_info.accel_x = (int16_t)((buf[1] << 8) | buf[0]);
    info->mpu_info.accel_y = (int16_t)((buf[3] << 8) | buf[2]);
    info->mpu_info.accel_z = (int16_t)((buf[5] << 8) | buf[4]);

    /* converts the accelerator data */
    info->accel[0] = BMI088_ACCEL_SEN * info->mpu_info.accel_x;
    info->accel[1] = BMI088_ACCEL_SEN * info->mpu_info.accel_y;
    info->accel[2] = BMI088_ACCEL_SEN * info->mpu_info.accel_z;

    /* read the temperature */
    readRegisters(Bmi088Sensor::ACCEL, BMI088_TEMP_M, buf, 2);
    info->mpu_info.temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (info->mpu_info.temperature > 1023) {
        info->mpu_info.temperature -= 2048;
    }

    /* converts the temperature data */
    info->temperature = info->mpu_info.temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    /* read the gyro multi data */
    readRegisters(Bmi088Sensor::GYRO, BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        info->mpu_info.gyro_x = (int16_t)((buf[3]) << 8) | buf[2];
        info->mpu_info.gyro_y = (int16_t)((buf[5]) << 8) | buf[4];
        info->mpu_info.gyro_z = (int16_t)((buf[7]) << 8) | buf[6];
    }

    /* converts the gyro data */
    info->gyro[0] = BMI088_GYRO_SEN * info->mpu_info.gyro_x - info->offsets_gyro_x;
    info->gyro[1] = BMI088_GYRO_SEN * info->mpu_info.gyro_y - info->offsets_gyro_y;
    info->gyro[2] = BMI088_GYRO_SEN * info->mpu_info.gyro_z - info->offsets_gyro_z;
}

bool bmi088AccelDataError(void)
{
    return !std::isfinite(bmi088_info.accel[0]) ||
           !std::isfinite(bmi088_info.accel[1]) ||
           !std::isfinite(bmi088_info.accel[2]);
}

bool bmi088GyroDataError(void)
{
    return !std::isfinite(bmi088_info.gyro[0]) ||
           !std::isfinite(bmi088_info.gyro[1]) ||
           !std::isfinite(bmi088_info.gyro[2]);
}