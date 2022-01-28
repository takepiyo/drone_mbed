/*
The MIT License (MIT)

Copyright (c) 2013 Seeed Technology Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef _BMM150_H_
#define _BMM150_H_

/** Includes */
// #include <Arduino.h>
// #include <Wire.h>
#include <geometry_msgs/Vector3.h>
#include <ros.h>
#include "bmm150_defs.h"
#include "mbed.h"

class BMM150 {
public:
  BMM150(PinName sda, PinName scl);
  /**
      \brief initialze device

  */
  int8_t initialize(void);

  /**
      \brief Read magnetometer data
  */
  geometry_msgs::Vector3 read_mag_data();

  /**
      @brief This internal API is used to obtain the compensated
      magnetometer x axis data(micro-tesla) in float.
  */
  int16_t compensate_x(int16_t mag_data_z, uint16_t data_rhall);

  /**
      @brief This internal API is used to obtain the compensated
      magnetometer Y axis data(micro-tesla) in int16_t.
  */
  int16_t compensate_y(int16_t mag_data_z, uint16_t data_rhall);

  /**
      @brief This internal API is used to obtain the compensated
      magnetometer Z axis data(micro-tesla) in int16_t.
  */
  int16_t compensate_z(int16_t mag_data_z, uint16_t data_rhall);

  /**
      \brief Set power mode
  */
  void set_op_mode(uint8_t op_mode);

  /**
      @brief This internal API reads the trim registers of the sensor and stores
      the trim values in the "trim_data" of device structure.
  */
  void read_trim_registers();

  /**
      @brief This internal API writes the op_mode value in the Opmode bits
      (bits 1 and 2) of 0x4C register.
  */
  void write_op_mode(uint8_t op_mode);

  /**
      \brief Set preset mode mode
  */
  void set_preset_mode(uint8_t mode);

  /**
      @brief This internal API sets/resets the power control bit of 0x4B
     register.
  */
  void set_power_control_bit(uint8_t pwrcntrl_bit);

  /**
      @brief This internal API sets the device from suspend to sleep mode
      by setting the power control bit to '1' of 0x4B register
  */
  void suspend_to_sleep_mode();

  /**
      @brief This API is used to set the preset mode of the sensor.
  */
  void set_presetmode(uint8_t preset_mode);

  /**
      Self test functionality
  */
  /*
      int8_t perform_self_test(uint8_t self_test_mode);
      int8_t perform_normal_self_test();
      void enable_normal_self_test(uint8_t *self_test_enable);
      int8_t validate_normal_self_test();
      int8_t perform_adv_self_test();
      void adv_self_test_settings();
      void adv_self_test_measurement(uint8_t self_test_current, int16_t
     *data_z); int8_t validate_adv_self_test(int16_t positive_data_z, int16_t
     negative_data_z); void set_adv_self_test_current(uint8_t
     self_test_current); void set_control_measurement_xyz(struct bmm150_settings
     settings);
  */

  /**
      @brief This internal API sets the preset mode ODR and repetition settings.
  */
  void set_odr_xyz_rep(struct bmm150_settings settings);

  /**
      @brief This internal API sets the xy repetition value in the 0x51
     register.
  */
  void set_xy_rep(struct bmm150_settings settings);

  /**
      @brief This internal API sets the z repetition value in the 0x52 register.
  */
  void set_z_rep(struct bmm150_settings settings);

  /**
      @brief This internal API is used to set the output data rate of the
     sensor.
  */
  void set_odr(struct bmm150_settings settings);

  /**
      @brief This API is used to perform soft-reset of the sensor
      where all the registers are reset to their default values except 0x4B.
  */
  void soft_reset();

  void calibration();

  geometry_msgs::Vector3 get_mag_data();

  geometry_msgs::Vector3 get_offset();
  /**

  */
  //   char* getErrorText(short errorCode);

  // protected:
  struct bmm150_settings settings;
  struct bmm150_raw_mag_data raw_mag_data;
  struct bmm150_mag_data mag_data;
  //   struct bmm150_mag_data mag_offset_data;
  struct bmm150_trim_registers trim_data;
  struct bmm150_mag_double mag_offset;
  struct bmm150_mag_double ellipsoid_axis;

  void i2c_write(uint8_t address, uint8_t byte);
  void i2c_read(uint8_t address, uint8_t* buffer, short length);
  void i2c_read(uint8_t address, int8_t* buffer, short length);
  uint8_t i2c_read(uint8_t address);

  I2C i2c;
};

#endif
