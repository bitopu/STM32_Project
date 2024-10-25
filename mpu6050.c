#include "mpu6050.h"

enum MPU6050_Status mpu6050_init(I2C_HandleTypeDef hi2c){
	uint8_t response;
	HAL_I2C_Mem_Read(&hi2c,MPU_ADDR,MPU_ADDR_WHOAMI,1,&response,1,MPU_TIMEOUT);
	if (!(response == 0x68 || response == 0x98)){
		return MPU_ERR;
	}
	else{
		response = 0x00;
		mpu6050_filter_config(hi2c,MPU_LP_FILTER_184);
		HAL_I2C_Mem_Write(&hi2c,MPU_ADDR,MPU_ADDR_PWRMGMT1,1,&response,1,MPU_TIMEOUT);
		return MPU_OK;
	}
};
enum MPU6050_Status mpu6050_read_byte(I2C_HandleTypeDef hi2c, uint8_t reg_address, uint8_t *data){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c,MPU_ADDR,reg_address,1,data,1,MPU_TIMEOUT);
	return (status == HAL_OK)?MPU_OK:MPU_ERR;
};
enum MPU6050_Status mpu6050_write_byte(I2C_HandleTypeDef hi2c, uint8_t reg_address, uint8_t *data){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c,MPU_ADDR,reg_address,1,data,1,MPU_TIMEOUT);
	return (status == HAL_OK)?MPU_OK:MPU_ERR;
};
enum MPU6050_Status mpu6050_filter_config(I2C_HandleTypeDef hi2c, uint8_t Bandwidth){
	uint8_t reg_data;
	mpu6050_read_byte(hi2c,MPU_ADDR_CONFIG,&reg_data);
	reg_data &=~ (7);
	reg_data |= Bandwidth;
	return mpu6050_write_byte(hi2c,MPU_ADDR_CONFIG,&reg_data);
};
enum MPU6050_Status mpu6050_read_accelerometer(I2C_HandleTypeDef hi2c,accel_raw_data * raw_accel){
	uint8_t read_data[6];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c,MPU_ADDR,MPU_ADDR_ACCELOUT_BASE,1,read_data,6,MPU_TIMEOUT);
	raw_accel->x = ((int16_t)read_data[0]<<8|read_data[1]);
	raw_accel->y = ((int16_t)read_data[2]<<8|read_data[3]);
	raw_accel->z = ((int16_t)read_data[4]<<8|read_data[5]);
	return (status == HAL_OK) ? MPU_OK: MPU_ERR;
};
enum MPU6050_Status mpu6050_read_gyroscope(I2C_HandleTypeDef hi2c,gyro_raw_data * raw_gyro){
	uint8_t read_data[6];
	
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c,MPU_ADDR,MPU_ADDR_GYROOUT_BASE,1,read_data,6,MPU_TIMEOUT);
	raw_gyro->x = ((int16_t)read_data[0]<<8|read_data[1]);
	raw_gyro->y = ((int16_t)read_data[2]<<8|read_data[3]);
	raw_gyro->z = ((int16_t)read_data[4]<<8|read_data[5]);
	return (status == HAL_OK) ? MPU_OK: MPU_ERR;
};
enum MPU6050_Status mpu6050_calibrated(I2C_HandleTypeDef hi2c,gyro_raw_data * raw_gyro,accel_raw_data* raw_accel,calibrated_data_t * calib_data, uint8_t count_max){
	sum_t total = {0,0,0,0,0,0,0};
	enum MPU6050_Status status;
	while (total.count <= count_max){
		HAL_Delay(50);
		status = mpu6050_read_accelerometer(hi2c,raw_accel);
		status = mpu6050_read_gyroscope(hi2c,raw_gyro);
		if (status != MPU_OK) {
			Error_Handler();
		}
		total.count++;
		total.ax += raw_accel->x;
		total.ay += raw_accel->y;
		total.az += raw_accel->z-16384;
		total.wx += raw_gyro->x;
		total.wy += raw_gyro->y;
		total.wz += raw_gyro->z;			
	}
	calib_data->ax = (int16_t)(total.ax/total.count);
	calib_data->ay = (int16_t)(total.ay/total.count);
	calib_data->az = (int16_t)(total.az/total.count);
	calib_data->wx = (int16_t)(total.wx/total.count);
	calib_data->wy = (int16_t)(total.wy/total.count);
	calib_data->wz = (int16_t)(total.wz/total.count);
	
	return status;
	

};
void mpu6050_calib_data(gyro_raw_data * raw_gyro, accel_raw_data* raw_accel, calibrated_data_t* calib_data){
	raw_accel->x = raw_accel->x-calib_data->ax;
	raw_accel->y = raw_accel->y-calib_data->ay;
	raw_accel->z = raw_accel->z-calib_data->az;
	raw_gyro->x	 = raw_gyro->x-calib_data->wx;
	raw_gyro->y	 = raw_gyro->y-calib_data->wy;
	raw_gyro->z  = raw_gyro->z-calib_data->wz;
	

};
void mpu6050_kalman_filter(attitude_state_t * attitude,kalman_parameter_t * parameter,gyro_raw_data* raw_gyro,accel_raw_data* raw_accel,float dt){
	//1. Calculate pitch and roll from measurement
	float z[2];
	z[0] = atan2f((float)raw_accel->y,(float)raw_accel->z)*180/3.14;
	z[1] = -atan2f((float)raw_accel->x,sqrtf(powf((float)raw_accel->y,2)+powf((float)raw_accel->z,2)))*180/3.14;
	//2. Calcualte angular velocity in degree
	float w[2];
	w[0] = raw_gyro->x/131.00f;
	w[1] = raw_gyro->x/131.00f;
	//3. Extrapolate the state
	attitude->x[0] += dt*w[0];
	attitude->x[1] += dt*w[1];
	//4. Extrapolate uncertainty
	attitude->P[0] += parameter->Q[0];
	attitude->P[1] += parameter->Q[1];
	//5. Compute the Kalman Gain
	parameter->K[0]  = (attitude->P[0]*(attitude->P[1]+parameter->R[1]))/\
											((attitude->P[0]+parameter->R[0])*(attitude->P[1]+parameter->R[1]));
	parameter->K[1]  = (attitude->P[1]*(attitude->P[0]+parameter->R[0]))/\
											((attitude->P[0]+parameter->R[0])*(attitude->P[1]+parameter->R[1]));
	
	//6. Update the estimate with measurement
	attitude->x[0] += parameter->K[0]*(z[0]-attitude->x[0]);
	attitude->x[1] += parameter->K[0]*(z[1]-attitude->x[1]);
	//7. Update the estiamte uncertainty
	attitude->P[0] = (1-parameter->K[0])*attitude->P[0]*(1-parameter->K[0])+\
										(parameter->K[0]*parameter->R[0]*parameter->K[0]);
	attitude->P[1] = (1-parameter->K[1])*attitude->P[1]*(1-parameter->K[1])+\
										(parameter->K[1]*parameter->R[1]*parameter->K[1]);
	




};
uint8_t mpu6050_map(float value, uint8_t min_input, uint8_t max_input,  uint8_t min_output, uint8_t max_output){
	int8_t new_value = (int8_t) value;
	if (new_value < min_input) value = min_input;
	if (new_value > max_input) value = max_input;
	uint8_t output = (uint8_t)((value-min_input)/(max_input-min_input)*(max_output-min_input))+min_output;
	return output;

};
