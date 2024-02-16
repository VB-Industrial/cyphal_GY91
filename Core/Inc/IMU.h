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

void IMU_setup(void);
void IMU_process(void);

void imu_get_quat(vec_4ax *vector);
void imu_get_linear(vec_4ax *vector);
void imu_get_gyro(vec_4ax *vector);
void imu_get_euler(vec_4ax *vector);

vec_4ax createQuaternionMsgFromYaw(float yaw);

uint8_t I2C_byte_receive(uint8_t address);
uint8_t I2C_byte_transmit(uint8_t address, uint8_t byte);
uint8_t I2C_n_byte_receive(uint8_t address, uint8_t * buffer, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */
