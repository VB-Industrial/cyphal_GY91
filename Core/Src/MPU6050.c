/*
 * MPU6050.c
 *
 *  Created on: Feb 13, 2024
 *      Author: VR
 */

#include "MPU6050.h"

uint8_t bufAG[14] = {0};
uint16_t sAx,sAy,sAz,sGx,sGy,sGz; //axel-gyro components
int16_t xx;

int MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
	HAL_StatusTypeDef rv;
    uint8_t check = 0;
    uint8_t Data = 0;

    // check device ID WHO_AM_I

    rv = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        rv = HAL_I2C_Mem_Write_IT(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1);
        return 0;
    }
    return 1;
}

HAL_StatusTypeDef MPU6050_isReady(I2C_HandleTypeDef *I2Cx)
{
	 return HAL_I2C_IsDeviceReady(I2Cx, 0xD0, 2, 5);
}

void MPU6050_read(float *x, float *y,float *z, float  *vx, float *vy, float  *vz)
{
HAL_I2C_Mem_Read_IT(&hi2c4, 0xD0, 0x3B, 1, &bufAG[0], 14);

xx = ( bufAG[0] << 6) + (bufAG[1]>>2) + 8192; if (xx > 16383) {xx -= 16383;}; sAx=xx;
xx = ( bufAG[2] << 6) + (bufAG[3]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sAy=xx;
xx = ( bufAG[4] << 6) + (bufAG[5]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sAz=xx;
xx = ( bufAG[8] << 6) + (bufAG[9]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sGx=xx;
xx = ( bufAG[10] << 6) + (bufAG[11]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sGy=xx;
xx = ( bufAG[12] << 6) + (bufAG[13]>>2) + 8192; if (xx > 16383) { xx -= 16383;}; sGz=xx;

*x = (((float)(8192 - sAx))/ 163834) * 2 * (float)M_PI;
*y = (((float)(8192 - sAy))/ 16384) * 2 * (float)M_PI;
*z = (((float)(8192 - sAz))/ 16384) * 2 * (float)M_PI;
*vx = (((float)(8192 - sGx))/ 16384) * 2 * (float)M_PI;
*vy = (((float)(8192 - sGy))/ 16384) * 2 * (float)M_PI;
*vz = (((float)(8192 - sGz))/ 16384) * 2 * (float)M_PI;
}


