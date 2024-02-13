/*
 * MPU6050.h
 *
 *  Created on: Feb 13, 2024
 *      Author: VR
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "i2c.h"

int MPU6050_Init(I2C_HandleTypeDef *I2Cx);
HAL_StatusTypeDef MPU6050_isReady(I2C_HandleTypeDef *I2Cx);
void MPU6050_read(float *x, float *y,float *z, float  *vx, float *vy, float  *vz);


#endif /* INC_MPU6050_H_ */
