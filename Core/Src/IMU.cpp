/*
 * IMU.cpp
 *
 *  Created on: Nov 8, 2022
 *      Author: user
 */
#include "IMU.h"
#include "main.h"
#include "Adafruit_BNO055.h"

#include <cstring>

#define BNO055 0x29

Adafruit_BNO055 bno = Adafruit_BNO055(55);
uint8_t BNO_conf = 0;

imu::Quaternion quat;
imu::Vector<3> vec;

extern I2C_HandleTypeDef hi2c4;

namespace {
constexpr uint8_t BNO055_SNAPSHOT_START = Adafruit_BNO055::BNO055_ACCEL_DATA_X_LSB_ADDR;
constexpr size_t BNO055_SNAPSHOT_SIZE = 0x34U - BNO055_SNAPSHOT_START;

struct Bno055Snapshot {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t euler_h;
  int16_t euler_r;
  int16_t euler_p;
  int16_t quat_w;
  int16_t quat_x;
  int16_t quat_y;
  int16_t quat_z;
  int16_t linear_x;
  int16_t linear_y;
  int16_t linear_z;
  int16_t gravity_x;
  int16_t gravity_y;
  int16_t gravity_z;
};

Bno055Snapshot snapshot{};
uint8_t snapshot_raw[BNO055_SNAPSHOT_SIZE]{};
bool snapshot_valid = false;

static inline int16_t read_i16_le(const uint8_t* data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                             (static_cast<uint16_t>(data[1]) << 8));
}

void decode_snapshot() {
  snapshot.accel_x = read_i16_le(&snapshot_raw[0x00]);
  snapshot.accel_y = read_i16_le(&snapshot_raw[0x02]);
  snapshot.accel_z = read_i16_le(&snapshot_raw[0x04]);

  snapshot.mag_x = read_i16_le(&snapshot_raw[0x06]);
  snapshot.mag_y = read_i16_le(&snapshot_raw[0x08]);
  snapshot.mag_z = read_i16_le(&snapshot_raw[0x0A]);

  snapshot.gyro_x = read_i16_le(&snapshot_raw[0x0C]);
  snapshot.gyro_y = read_i16_le(&snapshot_raw[0x0E]);
  snapshot.gyro_z = read_i16_le(&snapshot_raw[0x10]);

  snapshot.euler_h = read_i16_le(&snapshot_raw[0x12]);
  snapshot.euler_r = read_i16_le(&snapshot_raw[0x14]);
  snapshot.euler_p = read_i16_le(&snapshot_raw[0x16]);

  snapshot.quat_w = read_i16_le(&snapshot_raw[0x18]);
  snapshot.quat_x = read_i16_le(&snapshot_raw[0x1A]);
  snapshot.quat_y = read_i16_le(&snapshot_raw[0x1C]);
  snapshot.quat_z = read_i16_le(&snapshot_raw[0x1E]);

  snapshot.linear_x = read_i16_le(&snapshot_raw[0x20]);
  snapshot.linear_y = read_i16_le(&snapshot_raw[0x22]);
  snapshot.linear_z = read_i16_le(&snapshot_raw[0x24]);

  snapshot.gravity_x = read_i16_le(&snapshot_raw[0x26]);
  snapshot.gravity_y = read_i16_le(&snapshot_raw[0x28]);
  snapshot.gravity_z = read_i16_le(&snapshot_raw[0x2A]);
}

void refresh_snapshot() {
  if (!BNO_conf) {
    return;
  }
  if (snapshot_valid) {
    return;
  }

  HAL_I2C_Mem_Read(&hi2c4, BNO055 << 1, BNO055_SNAPSHOT_START, 1, snapshot_raw, BNO055_SNAPSHOT_SIZE, 100);

  while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
  }

  decode_snapshot();
  snapshot_valid = true;
}
}  // namespace

void imu_begin_sample(void) {
  snapshot_valid = false;
}

void imu_get_quat(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    refresh_snapshot();
    const double scale = 1.0 / (1 << 14);
    quat = imu::Quaternion(
        scale * snapshot.quat_w,
        scale * snapshot.quat_x,
        scale * snapshot.quat_y,
        scale * snapshot.quat_z
    );
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
    refresh_snapshot();
    vec[0] = static_cast<double>(snapshot.linear_x) / 100.0;
    vec[1] = static_cast<double>(snapshot.linear_y) / 100.0;
    vec[2] = static_cast<double>(snapshot.linear_z) / 100.0;
    vector->x = vec.x();
    vector->y = vec.y();
    vector->z = vec.z();
  }
  return ;
}

void imu_get_gravity(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    refresh_snapshot();
    vec[0] = static_cast<double>(snapshot.gravity_x) / 100.0;
    vec[1] = static_cast<double>(snapshot.gravity_y) / 100.0;
    vec[2] = static_cast<double>(snapshot.gravity_z) / 100.0;
    vector->x = vec.x();
    vector->y = vec.y();
    vector->z = vec.z();
  }
  return ;
}

void imu_get_mag(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    refresh_snapshot();
    vec[0] = static_cast<double>(snapshot.mag_x) / 16.0;
    vec[1] = static_cast<double>(snapshot.mag_y) / 16.0;
    vec[2] = static_cast<double>(snapshot.mag_z) / 16.0;
    vector->x = vec.x();
    vector->y = vec.y();
    vector->z = vec.z();
  }
  return ;
}

void imu_get_accel(vec_4ax *vector)
{
  if( BNO_conf == 1)
  {
    refresh_snapshot();
    vec[0] = static_cast<double>(snapshot.accel_x) / 100.0;
    vec[1] = static_cast<double>(snapshot.accel_y) / 100.0;
    vec[2] = static_cast<double>(snapshot.accel_z) / 100.0;
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
    refresh_snapshot();
    vec[0] = static_cast<double>(snapshot.gyro_x) / 900.0;
    vec[1] = static_cast<double>(snapshot.gyro_y) / 900.0;
    vec[2] = static_cast<double>(snapshot.gyro_z) / 900.0;
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

bool imu_read_status(imu_status_t* status)
{
  if ((status == nullptr) || (BNO_conf != 1)) {
    return false;
  }

  uint8_t raw[6] = {};
  HAL_I2C_Mem_Read(
      &hi2c4,
      BNO055 << 1,
      Adafruit_BNO055::BNO055_CALIB_STAT_ADDR,
      1,
      raw,
      sizeof(raw),
      100
  );
  while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
  }

  status->calib_stat = raw[0];
  status->selftest_result = raw[1];
  status->intr_stat = raw[2];
  status->sys_clk_stat = raw[3];
  status->sys_stat = raw[4];
  status->sys_err = raw[5];
  return true;
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
    snapshot_valid = false;
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
  HAL_I2C_Mem_Read(&hi2c4, BNO055<<1, address, 1, &value, 1, 100);
  while( HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY ) {
  }

  return value;
}

uint8_t I2C_byte_transmit(uint8_t address, uint8_t byte)
{
  HAL_I2C_Mem_Write(&hi2c4, BNO055<<1, address, 1, &byte, 1, 100);
  HAL_Delay(1);

  return 0;
}

uint8_t I2C_n_byte_receive(uint8_t address, uint8_t * buffer, uint8_t len)
{
  HAL_I2C_Mem_Read(&hi2c4, BNO055<<1, address, 1, buffer, len, 100);
  while( HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY ) {
  }

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
  vec_4ax quat{};
  setRPY(0,0,yaw, &quat);
  return quat;
}
