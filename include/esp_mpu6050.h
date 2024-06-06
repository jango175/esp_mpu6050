#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

// #define MPU6050_I2C_INIT            1 // uncomment to initialize I2C driver

#define MPU6050_ADDR_0              0x68 // default I2C address
#define MPU6050_ADDR_1              0x69 // alternative I2C address

#define MPU6050_MAX_FREQ            400000
#define MPU6050_TIMEOUT_MS          1

// register map
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_FIFO_EN             0x23

#define MPU6050_ACCEL_XOUT_MSB      0x3B
#define MPU6050_ACCEL_XOUT_LSB      0x3C
#define MPU6050_ACCEL_YOUT_MSB      0x3D
#define MPU6050_ACCEL_YOUT_LSB      0x3E
#define MPU6050_ACCEL_ZOUT_MSB      0x3F
#define MPU6050_ACCEL_ZOUT_LSB      0x40
#define MPU6050_TEMP_OUT_MSB        0x41
#define MPU6050_TEMP_OUT_LSB        0x42
#define MPU6050_GYRO_XOUT_MSB       0x43
#define MPU6050_GYRO_XOUT_LSB       0x44
#define MPU6050_GYRO_YOUT_MSB       0x45
#define MPU6050_GYRO_YOUT_LSB       0x46
#define MPU6050_GYRO_ZOUT_MSB       0x47
#define MPU6050_GYRO_ZOUT_LSB       0x48

#define MPU6050_PWR_MGMT_1          0x6B

enum mpu6050_ext_sync_set
{
  MPU6050_EXT_SYNC_SET_DISABLED = 0,
  MPU6050_EXT_SYNC_SET_TEMP_OUT_L,
  MPU6050_EXT_SYNC_SET_GYRO_XOUT_L,
  MPU6050_EXT_SYNC_SET_GYRO_YOUT_L,
  MPU6050_EXT_SYNC_SET_GYRO_ZOUT_L,
  MPU6050_EXT_SYNC_SET_ACCEL_XOUT_L,
  MPU6050_EXT_SYNC_SET_ACCEL_YOUT_L,
  MPU6050_EXT_SYNC_SET_ACCEL_ZOUT_L
};

enum mpu6050_dlpf_cfg
{
  MPU6050_DLPF_CFG_0 = 0,
  MPU6050_DLPF_CFG_1,
  MPU6050_DLPF_CFG_2,
  MPU6050_DLPF_CFG_3,
  MPU6050_DLPF_CFG_4,
  MPU6050_DLPF_CFG_5,
  MPU6050_DLPF_CFG_6,
  MPU6050_DLPF_CFG_7
};

enum mpu6050_fs_sel
{
  MPU6050_FS_SEL_250 = 0,
  MPU6050_FS_SEL_500,
  MPU6050_FS_SEL_1000,
  MPU6050_FS_SEL_2000
};

enum mpu6050_afs_sel
{
  MPU6050_AFS_SEL_2G = 0,
  MPU6050_AFS_SEL_4G,
  MPU6050_AFS_SEL_8G,
  MPU6050_AFS_SEL_16G
};

enum mpu6050_clk_sel
{
  MPU6050_CLKSEL_INTERNAL = 0,
  MPU6050_CLKSEL_PLL_XGYRO,
  MPU6050_CLKSEL_PLL_YGYRO,
  MPU6050_CLKSEL_PLL_ZGYRO,
  MPU6050_CLKSEL_PLL_EXT32K,
  MPU6050_CLKSEL_PLL_EXT19M,
  MPU6050_CLKSEL_RESERVED,
  MPU6050_CLKSEL_STOP
};


typedef struct mpu6050_conf_t
{
  i2c_port_num_t i2c_port;
  gpio_num_t sda_pin;
  gpio_num_t scl_pin;
  uint32_t i2c_freq;
  uint8_t i2c_addr;

  enum mpu6050_fs_sel fs_sel;
  enum mpu6050_afs_sel afs_sel;
} mpu6050_conf_t;


void mpu6050_init(mpu6050_conf_t mpu);

void mpu6050_set_sample_rate_divider(mpu6050_conf_t mpu, uint8_t divider);

void mpu6050_set_config(mpu6050_conf_t mpu, enum mpu6050_ext_sync_set ext_sync_set, enum mpu6050_dlpf_cfg dlpf_cfg);

void mpu6050_set_gyroscope_config(mpu6050_conf_t mpu, bool xg_st, bool yg_st, bool zg_st, enum mpu6050_fs_sel fs_sel);

void mpu6050_set_accelerometer_config(mpu6050_conf_t mpu, bool xa_st, bool ya_st, bool za_st,
                                        enum mpu6050_afs_sel afs_sel);

void mpu6050_set_fifo_enable(mpu6050_conf_t mpu, bool enable_temp, bool enable_gyro, bool enable_accel,
                              bool enable_slave2, bool enable_slave1, bool enable_slave0);

void mpu6050_i2c_passthrough(mpu6050_conf_t mpu);

void mpu6050_read_raw_accelerometer(mpu6050_conf_t mpu, int16_t* x, int16_t* y, int16_t* z);

void mpu6050_read_raw_temperature(mpu6050_conf_t mpu, int16_t* temp);

void mpu6050_read_raw_gyroscope(mpu6050_conf_t mpu, int16_t* x, int16_t* y, int16_t* z);

void mpu6050_read_accelerometer(mpu6050_conf_t mpu, float* x, float* y, float* z);

void mpu6050_read_temperature(mpu6050_conf_t mpu, float* temp);

void mpu6050_read_gyroscope(mpu6050_conf_t mpu, float* x, float* y, float* z);

void mpu6050_set_power(mpu6050_conf_t mpu, bool dev_reset, bool sleep, bool cycle, bool temp_dis,
                        enum mpu6050_clk_sel clk_sel);