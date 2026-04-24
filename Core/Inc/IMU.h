/*
 * IMU.h
 *
 *  Created on: Nov 8, 2022
 *      Author: user
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float x;
  float y;
  float z;
  float w;
} vec_4ax;

typedef struct
{
  uint8_t calib_stat;
  uint8_t selftest_result;
  uint8_t intr_stat;
  uint8_t sys_clk_stat;
  uint8_t sys_stat;
  uint8_t sys_err;
} imu_status_t;

void IMU_setup(void);
void IMU_process(void);
void imu_begin_sample(void);

void imu_get_quat(vec_4ax *vector);
void imu_get_linear(vec_4ax *vector);
void imu_get_gravity(vec_4ax *vector);
void imu_get_accel(vec_4ax *vector);
void imu_get_mag(vec_4ax *vector);
void imu_get_gyro(vec_4ax *vector);
void imu_get_euler(vec_4ax *vector);
bool imu_read_status(imu_status_t* status);

vec_4ax createQuaternionMsgFromYaw(float yaw);

uint8_t I2C_byte_receive(uint8_t address);
uint8_t I2C_byte_transmit(uint8_t address, uint8_t byte);
uint8_t I2C_n_byte_receive(uint8_t address, uint8_t * buffer, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */
