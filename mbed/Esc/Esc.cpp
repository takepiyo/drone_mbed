#include "mbed.h"
#include <ros.h>
#include <Esc.h>

Esc::Esc(PinName pwm_pin) : PwmOut(pwm_pin)
{
  this->period_ms(PERIOD);
  this->write(INITIAL_DUTY);
}

Esc::~Esc()
{

}

void Esc::update(float input)
{
   _write(input);
}

void Esc::stop()
{
  this->write(0.0);
}

void Esc::_write(float raw_duty)
{
  if(raw_duty < 0)
  {
    raw_duty = 0;
  }
  float valid_duty = (MAX_DUTY - MIN_DUTY) * raw_duty + MIN_DUTY;
  this->write(valid_duty);
}