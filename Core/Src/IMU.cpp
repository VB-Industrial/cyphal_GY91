/*
 * IMU.cpp
 *
 *  Created on: Nov 8, 2022
 *      Author: user
 */
#include "IMU.h"
#include "main.h"
#include "Adafruit_BNO055.h"

#define BNO055 0x29

Adafruit_BNO055 bno = Adafruit_BNO055(55);
uint8_t BNO_conf = 0;

imu::Quaternion quat;
imu::Vector<3> vec;

void imu_get_quat(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    quat = bno.getQuat();
    vector->x = quat.x();
    vector->y = quat.y();
    vector->z = quat.z();
    vector->w = quat.w();
  }
  return ;
}

void imu_get_linear(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    vec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    vector->x = vec.x();
    vector->y = vec.y();
    vector->z = vec.z();
  }
  return ;
}

void imu_get_gyro(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    vector->x = vec.x();
    vector->y = vec.y();
    vector->z = vec.z();
  }
  return ;
}

void imu_get_euler(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    vector->x = vec.x();
    vector->y = vec.y();
    vector->z = vec.z();
  }
  return ;
}

void IMU_setup(void)
{
//  HAL_Delay(5);
//  HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
//  HAL_Delay(5);
//  HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
//  HAL_Delay(50);

  BNO_conf = 0;
  if(bno.begin())
  {
    BNO_conf = 1;
  }
  else
  {
    IMU_setup();
  }
  return ;
}

void IMU_process(void)
{
  if( BNO_conf == 1)
  {
    vec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  }
}

extern I2C_HandleTypeDef hi2c4;

uint8_t I2C_byte_receive(uint8_t address)
{
  uint8_t value = 0;
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c4, BNO055<<1, address, 1, &value, 1, 100);
  while( HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY ){}

  return value;
}

uint8_t I2C_byte_transmit(uint8_t address, uint8_t byte)
{
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, BNO055<<1, address, 1, &byte, 1, 100);
  HAL_Delay(1);

  return 0;
}

uint8_t I2C_n_byte_receive(uint8_t address, uint8_t * buffer, uint8_t len)
{
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c4, BNO055<<1, address, 1, buffer, len, 100);
  while( HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY ){}

  return 0;
}

void setRPY(float roll, float pitch, float yaw, vec_4ax *vector)
{
  float halfYaw = yaw * 0.5f;
  float halfPitch = pitch * 0.5f;
  float halfRoll = roll * 0.5f;
  float cosYaw = cos(halfYaw);
  float sinYaw = sin(halfYaw);
  float cosPitch = cos(halfPitch);
  float sinPitch = sin(halfPitch);
  float cosRoll = cos(halfRoll);
  float sinRoll = sin(halfRoll);

  vector->x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  vector->y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  vector->z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  vector->w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
}

vec_4ax createQuaternionMsgFromYaw(float yaw)
{
  vec_4ax quat = {0};
  setRPY(0,0,yaw, &quat);
  return quat;
}
