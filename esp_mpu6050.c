#include <stdio.h>
#include "esp_mpu6050.h"

#ifdef HMC5883L_I2C_INIT
static i2c_master_bus_handle_t bus_handle;
#else
extern i2c_master_bus_handle_t bus_handle;
#endif
static i2c_master_dev_handle_t dev_handle;


/**
 * @brief i2c write handler
 * 
 * @param mpu struct with MPU6050 parameters
 * @param data pointer to the array for data to write
 * @param data_len length of data to write
*/
static void i2c_write(mpu6050_conf_t mpu, uint8_t* data, uint32_t data_len)
{
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, data_len, MPU6050_TIMEOUT_MS));
}


/**
 * @brief i2c read and write handler
 * 
 * @param mpu struct with MPU6050 parameters
 * @param reg address of register to read
 * @param data pointer to the array for read data
 * @param data_len length of data to read
*/
static void i2c_write_read(mpu6050_conf_t mpu, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, data_len, MPU6050_TIMEOUT_MS));
}


/**
 * @brief initialize MPU6050
 * 
 * @param mpu struct with MPU6050 parameters
*/
void mpu6050_init(mpu6050_conf_t mpu)
{
#ifdef MPU6050_I2C_INIT
    if (mpu.i2c_freq > MPU6050_MAX_FREQ)
    {
        mpu.i2c_freq = MPU6050_MAX_FREQ;
        printf("QMC5883L: I2C frequency too high, set to value: %d Hz\n", MPU6050_MAX_FREQ);
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = mpu.i2c_port,
        .scl_io_num = mpu.scl_pin,
        .sda_io_num = mpu.sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
#endif

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mpu.i2c_addr,
        .scl_speed_hz = mpu.i2c_freq
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    mpu6050_set_power(mpu, false, false, false, true, MPU6050_CLKSEL_INTERNAL);
    mpu6050_set_sample_rate_divider(mpu, 0);
    mpu6050_set_config(mpu, MPU6050_EXT_SYNC_SET_DISABLED, MPU6050_DLPF_CFG_0);
    mpu6050_set_gyroscope_config(mpu, false, false, false, MPU6050_FS_SEL_500);
    mpu6050_set_accelerometer_config(mpu, false, false, false, MPU6050_AFS_SEL_4G);
    mpu6050_set_fifo_enable(mpu, false, true, true, false, false, false);
}


/**
 * @brief set sample rate divider
 * 
 * @param mpu struct with MPU6050 parameters
 * @param divider sample rate divider (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
*/
void mpu6050_set_sample_rate_divider(mpu6050_conf_t mpu, uint8_t divider)
{
    uint8_t data[2] = {MPU6050_SMPLRT_DIV, divider};
    i2c_write(mpu, data, 2);
}


/**
 * @brief set MPU6050 configuration
 * 
 * @param mpu struct with MPU6050 parameters
 * @param ext_sync_set FSYNC configuration
 * @param dlpf_cfg Digital Low Pass Filter configuration
*/
void mpu6050_set_config(mpu6050_conf_t mpu, enum mpu6050_ext_sync_set ext_sync_set, enum mpu6050_dlpf_cfg dlpf_cfg)
{
    uint8_t old_config = 0;
    i2c_write_read(mpu, MPU6050_CONFIG, &old_config, 1);

    uint8_t config = (old_config & 0b11000000) | (ext_sync_set << 3) | dlpf_cfg;

    uint8_t data[2] = {MPU6050_CONFIG, config};
    i2c_write(mpu, data, 2);
}


/**
 * @brief set gyroscope configuration
 * 
 * @param mpu struct with MPU6050 parameters
 * @param xg_st X-axis gyroscope self-test
 * @param yg_st Y-axis gyroscope self-test
 * @param zg_st Z-axis gyroscope self-test
 * @param fs_sel gyroscope full scale range
*/
void mpu6050_set_gyroscope_config(mpu6050_conf_t mpu, bool xg_st, bool yg_st, bool zg_st, enum mpu6050_fs_sel fs_sel)
{
    uint8_t old_config = 0;
    i2c_write_read(mpu, MPU6050_GYRO_CONFIG, &old_config, 1);

    uint8_t config = ((uint8_t)xg_st << 7) | ((uint8_t)yg_st << 6) | ((uint8_t)zg_st << 5) |
                        (fs_sel << 3) | (old_config & 0b00000111);

    uint8_t data[2] = {MPU6050_GYRO_CONFIG, config};
    i2c_write(mpu, data, 2);

    mpu.fs_sel = fs_sel;
}


/**
 * @brief set accelerometer configuration
 * 
 * @param mpu struct with MPU6050 parameters
 * @param xa_st X-axis accelerometer self-test
 * @param ya_st Y-axis accelerometer self-test
 * @param za_st Z-axis accelerometer self-test
 * @param afs_sel accelerometer full scale range
*/
void mpu6050_set_accelerometer_config(mpu6050_conf_t mpu, bool xa_st, bool ya_st, bool za_st,
                                        enum mpu6050_afs_sel afs_sel)
{
    uint8_t old_config = 0;
    i2c_write_read(mpu, MPU6050_ACCEL_CONFIG, &old_config, 1);

    uint8_t config = ((uint8_t)xa_st << 7) | ((uint8_t)ya_st << 6) | ((uint8_t)za_st << 5) |
                        (afs_sel << 3) | (old_config & 0b00000111);

    uint8_t data[2] = {MPU6050_ACCEL_CONFIG, config};
    i2c_write(mpu, data, 2);

    mpu.afs_sel = afs_sel;
}


/**
 * set FIFO enable
 * 
 * @param mpu struct with MPU6050 parameters
 * @param enable_temp enable temperature sensor
 * @param enable_gyro enable gyroscope
 * @param enable_accel enable accelerometer
 * @param enable_slave2 enable external sensor 2
 * @param enable_slave1 enable external sensor 1
 * @param enable_slave0 enable external sensor 0
*/
void mpu6050_set_fifo_enable(mpu6050_conf_t mpu, bool enable_temp, bool enable_gyro, bool enable_accel,
                              bool enable_slave2, bool enable_slave1, bool enable_slave0)
{
    uint8_t enable = ((uint8_t)enable_temp << 7) | ((uint8_t)enable_gyro << 6) | ((uint8_t)enable_gyro << 5) |
                     ((uint8_t)enable_gyro << 4) | ((uint8_t)enable_accel << 3) | ((uint8_t)enable_slave2 << 2) |
                     ((uint8_t)enable_slave1 << 1) | (uint8_t)enable_slave0;

    uint8_t data[2] = {MPU6050_FIFO_EN, enable};
    i2c_write(mpu, data, 2);
}


/**
 * @brief set MPU6050 to I2C passthrough mode
 * 
 * @param mpu struct with MPU6050 parameters
*/
void mpu6050_i2c_passthrough(mpu6050_conf_t mpu)
{
    uint8_t data[2] = {0x6A, 0};
    i2c_write(mpu, data, 2);

    data[0] = 0x37;
    data[1] = 2;
    i2c_write(mpu, data, 2);

    data[0] = 0x6B;
    data[1] = 0;
    i2c_write(mpu, data, 2);
}


/**
 * @brief read raw accelerometer data
 * 
 * @param mpu struct with MPU6050 parameters
 * @param x pointer to x-axis data
 * @param y pointer to y-axis data
 * @param z pointer to z-axis data
*/
void mpu6050_read_raw_accelerometer(mpu6050_conf_t mpu, int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t data[6];
    i2c_write_read(mpu, MPU6050_ACCEL_XOUT_MSB, data, 6);

    *x = (data[0] << 8) | data[1];
    *y = (data[2] << 8) | data[3];
    *z = (data[4] << 8) | data[5];
}


/**
 * @brief read raw temperature data
 * 
 * @param mpu struct with MPU6050 parameters
 * @param temp pointer to temperature data
*/
void mpu6050_read_raw_temperature(mpu6050_conf_t mpu, int16_t* temp)
{
    uint8_t data[2];
    i2c_write_read(mpu, MPU6050_TEMP_OUT_MSB, data, 2);

    *temp = (data[0] << 8) | data[1];
}


/**
 * @brief read raw gyroscope data
 * 
 * @param mpu struct with MPU6050 parameters
 * @param x pointer to x-axis data
 * @param y pointer to y-axis data
 * @param z pointer to z-axis data
*/
void mpu6050_read_raw_gyroscope(mpu6050_conf_t mpu, int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t data[6];
    i2c_write_read(mpu, MPU6050_GYRO_XOUT_MSB, data, 6);

    *x = (data[0] << 8) | data[1];
    *y = (data[2] << 8) | data[3];
    *z = (data[4] << 8) | data[5];
}


/**
 * @brief read accelerometer
 * 
 * @param mpu struct with MPU6050 parameters
 * @param x pointer to x-axis data
 * @param y pointer to y-axis data
 * @param z pointer to z-axis data
*/
void mpu6050_read_accelerometer(mpu6050_conf_t mpu, float* x, float* y, float* z)
{
    int16_t raw_x, raw_y, raw_z;
    mpu6050_read_raw_accelerometer(mpu, &raw_x, &raw_y, &raw_z);

    float afs_sel = 16384.0f;

    switch (mpu.afs_sel)
    {
        case MPU6050_AFS_SEL_2G:
            afs_sel = 16384.0f;
            break;
        case MPU6050_AFS_SEL_4G:
            afs_sel = 8192.0f;
            break;
        case MPU6050_AFS_SEL_8G:
            afs_sel = 4096.0f;
            break;
        case MPU6050_AFS_SEL_16G:
            afs_sel = 2048.0f;
            break;
    }

    *x = (float)raw_x / afs_sel;
    *y = (float)raw_y / afs_sel;
    *z = (float)raw_z / afs_sel;
}


/**
 * @brief read temperature
 * 
 * @param mpu struct with MPU6050 parameters
 * @param temp pointer to temperature data
*/
void mpu6050_read_temperature(mpu6050_conf_t mpu, float* temp)
{
    int16_t raw_temp;
    mpu6050_read_raw_temperature(mpu, &raw_temp);

    *temp = (float)raw_temp / 340.0f + 36.53f;
}


/**
 * @brief read gyroscope
 * 
 * @param mpu struct with MPU6050 parameters
 * @param x pointer to x-axis data
 * @param y pointer to y-axis data
 * @param z pointer to z-axis data
*/
void mpu6050_read_gyroscope(mpu6050_conf_t mpu, float* x, float* y, float* z)
{
    int16_t raw_x, raw_y, raw_z;
    mpu6050_read_raw_gyroscope(mpu, &raw_x, &raw_y, &raw_z);

    float fs_sel = 131.0f;

    switch (mpu.fs_sel)
    {
        case MPU6050_FS_SEL_250:
            fs_sel = 131.0f;
            break;
        case MPU6050_FS_SEL_500:
            fs_sel = 65.5f;
            break;
        case MPU6050_FS_SEL_1000:
            fs_sel = 32.8f;
            break;
        case MPU6050_FS_SEL_2000:
            fs_sel = 16.4f;
            break;
    }

    *x = (float)raw_x / fs_sel;
    *y = (float)raw_y / fs_sel;
    *z = (float)raw_z / fs_sel;
}


/**
 * @brief set power management
 * 
 * @param mpu struct with MPU6050 parameters
 * @param dev_reset device reset
 * @param sleep sleep mode
 * @param cycle cycle mode
 * @param temp_dis temperature sensor disable
 * @param clk_sel clock select
*/
void mpu6050_set_power(mpu6050_conf_t mpu, bool dev_reset, bool sleep, bool cycle, bool temp_dis,
                        enum mpu6050_clk_sel clk_sel)
{
    uint8_t old_power = 0;
    i2c_write_read(mpu, MPU6050_PWR_MGMT_1, &old_power, 1);

    uint8_t power = ((uint8_t)dev_reset << 7) | ((uint8_t)sleep << 6) | ((uint8_t)cycle << 5) | (old_power & (1 << 4)) |
                    ((uint8_t)temp_dis << 3) | clk_sel;

    uint8_t data[2] = {MPU6050_PWR_MGMT_1, power};
    i2c_write(mpu, data, 2);
}
