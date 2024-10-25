#ifndef _MPU_6050_
#define _MPU_6050_
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include <math.h>
#define MPU_TIMEOUT				5000


#define MPU_ADDR								(0x68<<1)
#define MPU_ADDR_WHOAMI					117
#define MPU_ADDR_PWRMGMT1				107
#define MPU_ADDR_CONFIG					26
#define MPU_ADDR_ACCELOUT_BASE	59
#define MPU_ADDR_GYROOUT_BASE		67
#define MPU_ADDR_TEMPOUT_BASE		65


//Mode
//Filter
#define MPU_LP_FILTER_260	0
#define MPU_LP_FILTER_184	1
#define MPU_LP_FILTER_94	2
#define MPU_LP_FILTER_44	3
#define MPU_LP_FILTER_21  4
#define MPU_LP_FILTER_10	5
#define MPU_LP_FILTER_5		6
//
#define MIN_COUNT					0
#define MAX_COUNT					40
#define MIN_ANGLE					0
#define MAX_ANGLE					40

enum MPU6050_Status{
	MPU_OK,
	MPU_ERR
};
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
	
}accel_raw_data;
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
	
}gyro_raw_data;
typedef struct{
	int32_t ax;
	int32_t ay;
	int32_t az;
	int32_t wx;
	int32_t wy;
	int32_t wz;
	int32_t count;
	
}sum_t;
typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t wx;
	int16_t wy;
	int16_t wz;
}calibrated_data_t;
typedef struct	{
	float x[2];
	float P[2];
}attitude_state_t;
typedef struct	{
	float Q[2];
	float R[2];
	float K[2];
}kalman_parameter_t;
enum MPU6050_Status mpu6050_init(I2C_HandleTypeDef hi2c);
enum MPU6050_Status mpu6050_read_byte(I2C_HandleTypeDef hi2c, uint8_t reg_address, uint8_t *data);
enum MPU6050_Status mpu6050_write_byte(I2C_HandleTypeDef hi2c, uint8_t reg_address, uint8_t *data);
enum MPU6050_Status mpu6050_filter_config(I2C_HandleTypeDef hi2c, uint8_t Bandwidth);
enum MPU6050_Status mpu6050_read_accelerometer(I2C_HandleTypeDef hi2c,accel_raw_data * raw_accel);
enum MPU6050_Status mpu6050_read_gyroscope(I2C_HandleTypeDef hi2c,gyro_raw_data * raw_gyro);
enum MPU6050_Status mpu6050_calibrated(I2C_HandleTypeDef hi2c,gyro_raw_data * raw_gyro,accel_raw_data* raw_accel,calibrated_data_t * calib_data, uint8_t count_max);
void mpu6050_calib_data(gyro_raw_data * raw_gyro, accel_raw_data* raw_accel, calibrated_data_t* calib_data);
void mpu6050_kalman_filter(attitude_state_t * attitude,kalman_parameter_t * parameter,gyro_raw_data* raw_gyro,accel_raw_data* raw_accel,float dt);
uint8_t mpu6050_map(float value, uint8_t min_input, uint8_t max_input,  uint8_t min_output, uint8_t max_output);
#endif 
