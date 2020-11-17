#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32.h>

Esc::Esc(PinName pwm_pin)
{
  this->__motor(pwm_pin);
  _motor.period_ms(PERIOD);
  _motor.write(INITIAL_DUTY);
  this->_is_debug = false;
}

// Esc::Esc(PinName pwm_pin, ros::NodeHandle nh)
// {
//   this->_motor(pwm_pin);
//   _motor.period_ms(PERIOD);
//   _motor.write(INITIAL_DUTY);
//   this->_is_debug = true;
//   nh.advertise(this->_debugger)
// }

Esc::~Esc()
{

}

void Esc::update(std_msgs::Float32 input)
{
   this->_write(_input.data);
  //  if (this->is_debug)
  //  {
  //    this->_debugger.publish(&input);
  //  }
}

void Esc::stop()
{
  this->_motor.write(0.0);
}

void Esc::_write(float raw_duty)
{
  float valid_duty = (MAX_DUTY - MIN_DUTY) * raw_duty + MIN_DUTY;
  this->_motor.write(valid_duty);
}