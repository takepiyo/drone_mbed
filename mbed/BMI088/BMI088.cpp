/*
    A library for Grove - 6-Axis Accelerometer&Gyroscope（BMI088）

    Copyright (c) 2018 seeed technology co., ltd.
    Author      : Wayen Weng
    Create Time : June 2018
    Change Log  :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#include "BMI088.h"
#include "geometry_msgs/Vector3.h"
#include "ros.h"

BMI088::BMI088(PinName sda, PinName scl, float period) : i2c(sda, scl) {
  i2c.frequency(100000);

  acc_bias_x   = 0.0;
  acc_bias_y   = 0.0;
  acc_bias_z   = 0.0;
  this->period = period;
  // pre_acc_x = 0.0;
  // pre_acc_y = 0.0;
  // pre_acc_z = 9.79f;
  // tau = 1.0f / (2 * 3.141592f * CUT_OFF_FREQUENCY);
}

void BMI088::initialize(void) {
  setAccScaleRange(RANGE_6G);
  setAccOutputDataRate(ODR_100);
  setAccPoweMode(ACC_ACTIVE);

  setGyroScaleRange(RANGE_2000);
  setGyroOutputDataRate(ODR_2000_BW_532);
  setGyroPoweMode(GYRO_NORMAL);

  calibrationAcc();
}

bool BMI088::isConnection(void) {
  uint8_t val1 = getGyroID();
  uint8_t val2 = getAccID();
  return ((val1 == 0x0F) && (val2 == 0x1E));
  // return ((getAccID() == 0x1E) && (getGyroID() == 0x0F));
}

void BMI088::resetAcc(void) { write8(ACC, BMI088_ACC_SOFT_RESET, 0xB6); }

void BMI088::resetGyro(void) { write8(GYRO, BMI088_GYRO_SOFT_RESET, 0xB6); }

uint8_t BMI088::getAccID(void) { return read8(ACC, BMI088_GYRO_CHIP_ID); }

uint8_t BMI088::getGyroID(void) { return read8(GYRO, BMI088_GYRO_CHIP_ID); }

void BMI088::setAccPoweMode(acc_power_type_t mode) {
  if (mode == ACC_ACTIVE) {
    write8(ACC, BMI088_ACC_PWR_CTRl, 0x04);
    write8(ACC, BMI088_ACC_PWR_CONF, 0x00);
  } else if (mode == ACC_SUSPEND) {
    write8(ACC, BMI088_ACC_PWR_CONF, 0x03);
    write8(ACC, BMI088_ACC_PWR_CTRl, 0x00);
  }
}

void BMI088::setGyroPoweMode(gyro_power_type_t mode) {
  if (mode == GYRO_NORMAL) {
    write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
  } else if (mode == GYRO_SUSPEND) {
    write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
  } else if (mode == GYRO_DEEP_SUSPEND) {
    write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
  }
}

void BMI088::setAccScaleRange(acc_scale_type_t range) {
  if (range == RANGE_3G) {
    accRange = 3000;
  } else if (range == RANGE_6G) {
    accRange = 6000;
  } else if (range == RANGE_12G) {
    accRange = 12000;
  } else if (range == RANGE_24G) {
    accRange = 24000;
  }

  write8(ACC, BMI088_ACC_RANGE, (uint8_t)range);
}

void BMI088::setAccOutputDataRate(acc_odr_type_t odr) {
  uint8_t data = 0;

  data = read8(ACC, BMI088_ACC_CONF);
  data = data & 0xf0;
  data = data | (uint8_t)odr;

  write8(ACC, BMI088_ACC_CONF, data);
}

void BMI088::setGyroScaleRange(gyro_scale_type_t range) {
  if (range == RANGE_2000) {
    gyroRange = 2000;
  } else if (range == RANGE_1000) {
    gyroRange = 1000;
  } else if (range == RANGE_500) {
    gyroRange = 500;
  } else if (range == RANGE_250) {
    gyroRange = 250;
  } else if (range == RANGE_125) {
    gyroRange = 125;
  }

  write8(GYRO, BMI088_GYRO_RANGE, (uint8_t)range);
}

void BMI088::setGyroOutputDataRate(gyro_odr_type_t odr) {
  write8(GYRO, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}

geometry_msgs::Vector3 BMI088::getAcceleration() {
  geometry_msgs::Vector3 output;
  //    uint8_t buf[6] = {0};
  char buf[6] = {0};
  uint16_t ax = 0, ay = 0, az = 0;
  float value = 0;

  read(ACC, BMI088_ACC_X_LSB, buf, 6);

  ax = buf[0] | (buf[1] << 8);
  ay = buf[2] | (buf[3] << 8);
  az = buf[4] | (buf[5] << 8);

  value = (int16_t)ax;
  // *x = accRange * value / 32768000 * 9.79f - acc_bias_x; // units: m/s^2
  output.x = accRange * value / 32768000 * 9.79f - acc_bias_x;  // units: m/s^2

  value = (int16_t)ay;
  // *x = accRange * value / 32768000 * 9.79f - acc_bias_x; // units: m/s^2
  output.y = accRange * value / 32768000 * 9.79f - acc_bias_y;  // units: m/s^2

  value = (int16_t)az;
  // *x = accRange * value / 32768000 * 9.79f - acc_bias_x; // units: m/s^2
  output.z = accRange * value / 32768000 * 9.79f - acc_bias_z;  // units: m/s^2

  // low pass filter
  // *x = (tau * pre_acc_x) / (tau + period) + (period * *x) / (tau + period);
  // *y = (tau * pre_acc_y) / (tau + period) + (period * *y) / (tau + period);
  // *z = (tau * pre_acc_z) / (tau + period) + (period * *z) / (tau + period);
  return output;
}

float BMI088::getAccelerationX(void) {
  uint16_t ax = 0;
  float value = 0;

  ax = read16(ACC, BMI088_ACC_X_LSB);

  value = (int16_t)ax;
  value = accRange * value / 32768000 * 9.79f;  // units: m/s^2

  return value;
}

float BMI088::getAccelerationY(void) {
  uint16_t ay = 0;
  float value = 0;

  ay = read16(ACC, BMI088_ACC_Y_LSB);

  value = (int16_t)ay;
  value = accRange * value / 32768000 * 9.79f;  // units: m/s^2

  return value;
}

float BMI088::getAccelerationZ(void) {
  uint16_t az = 0;
  float value = 0;

  az = read16(ACC, BMI088_ACC_Z_LSB);

  value = (int16_t)az;
  value = accRange * value / 32768000 * 9.79f;  // units: m/s^2

  return value;
}

geometry_msgs::Vector3 BMI088::getGyroscope() {
  geometry_msgs::Vector3 output;
  //    uint8_t buf[6] = {0};
  char buf[6] = {0};
  uint16_t gx = 0, gy = 0, gz = 0;
  float value = 0;

  read(GYRO, BMI088_GYRO_RATE_X_LSB, buf, 6);

  gx = buf[0] | (buf[1] << 8);
  gy = buf[2] | (buf[3] << 8);
  gz = buf[4] | (buf[5] << 8);

  value = (int16_t)gx;
  // *x = gyroRange * value / 32768 * 3.1415927f / 180.0f;
  output.x = gyroRange * value / 32768 * 3.1415927f / 180.0f;
  value    = (int16_t)gy;
  // *y = gyroRange * value / 32768 * 3.1415927f / 180.0f;
  output.y = gyroRange * value / 32768 * 3.1415927f / 180.0f;
  value    = (int16_t)gz;
  // *z = gyroRange * value / 32768 * 3.1415927f / 180.0f;
  output.z = gyroRange * value / 32768 * 3.1415927f / 180.0f;
  return output;
}

float BMI088::getGyroscopeX(void) {
  uint16_t gx = 0;
  float value = 0;

  gx = read16(GYRO, BMI088_GYRO_RATE_X_LSB);

  value = (int16_t)gx;
  value = gyroRange * value / 32768 * 3.1415927f / 180.0f;

  return value;
}

float BMI088::getGyroscopeY(void) {
  uint16_t gy = 0;
  float value = 0;

  gy = read16(GYRO, BMI088_GYRO_RATE_Y_LSB);

  value = (int16_t)gy;
  value = gyroRange * value / 32768 * 3.1415927f / 180.0f;

  return value;
}

float BMI088::getGyroscopeZ(void) {
  uint16_t gz = 0;
  float value = 0;

  gz = read16(GYRO, BMI088_GYRO_RATE_Z_LSB);

  value = (int16_t)gz;
  value = gyroRange * value / 32768 * 3.1415927f / 180.0f;

  return value;
}

int16_t BMI088::getTemperature(void) {
  uint16_t data = 0;

  data = read16Be(ACC, BMI088_ACC_TEMP_MSB);
  data = data >> 5;

  if (data > 1023) { data = data - 2048; }

  return (int16_t)(data / 8 + 23);
}

void BMI088::write8(device_type_t dev, char reg, uint8_t val) {
  char cmd[2];
  cmd[0] = reg;
  cmd[1] = val;

  int addr = 0;

  if (dev) {
    addr = BMI088_GYRO_ADDRESS;
  } else {
    addr = BMI088_ACC_ADDRESS;
  }

  i2c.write(addr, cmd, 2);
}

uint8_t BMI088::read8(device_type_t dev, char reg) {
  int addr = 0;
  char data;

  if (dev) {
    addr = BMI088_GYRO_ADDRESS;
  } else {
    addr = BMI088_ACC_ADDRESS;
  }

  i2c.write(addr, &reg, 1);
  i2c.read(addr, &data, 1);

  return data;
}

uint16_t BMI088::read16(device_type_t dev, char reg) {
  int addr = 0;
  // uint16_t msb = 0, lsb = 0;
  char data[2];

  if (dev) {
    addr = BMI088_GYRO_ADDRESS;
  } else {
    addr = BMI088_ACC_ADDRESS;
  }

  i2c.write(addr, &reg, 1);
  i2c.read(addr, data, 2);

  return (data[0] | (data[1] << 8));
}

uint16_t BMI088::read16Be(device_type_t dev, char reg) {
  uint8_t addr = 0;
  // uint16_t msb = 0, lsb = 0;

  char data[2];

  if (dev) {
    addr = BMI088_GYRO_ADDRESS;
  } else {
    addr = BMI088_ACC_ADDRESS;
  }

  i2c.write(addr, &reg, 1);
  i2c.read(addr, data, 2);

  return (data[0] | (data[1] << 8));
}

uint32_t BMI088::read24(device_type_t dev, char reg) {
  int addr = 0;
  char data[3];

  if (dev) {
    addr = BMI088_GYRO_ADDRESS;
  } else {
    addr = BMI088_ACC_ADDRESS;
  }

  i2c.write(addr, &reg, 1);
  i2c.read(addr, data, 3);

  return (data[0] | (data[1] << 8) | (data[2] << 16));
}

void BMI088::read(device_type_t dev, char reg, char *buf, uint16_t len) {
  uint8_t addr = 0;
  if (dev) {
    addr = BMI088_GYRO_ADDRESS;
  } else {
    addr = BMI088_ACC_ADDRESS;
  }

  i2c.write(addr, &reg, 1);
  i2c.read(addr, buf, len);
}

void BMI088::calibrationAcc(void) {
  float x_ave = 0.0, y_ave = 0.0, z_ave = 0.0;
  for (int i = 0; i < CALIBRATION_COUNT; i++) {
    geometry_msgs::Vector3 acc;
    acc = getAcceleration();
    x_ave += acc.x / CALIBRATION_COUNT;
    y_ave += acc.y / CALIBRATION_COUNT;
    z_ave += (acc.z - 9.79f) / CALIBRATION_COUNT;
    wait(period * 10);
  }
  acc_bias_x = x_ave;
  acc_bias_y = y_ave;
  acc_bias_z = z_ave;
}