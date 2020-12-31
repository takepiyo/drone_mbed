#include <ros.h>
#include "mbed.h"

#define PERIOD 10
#define MIN_DUTY 0.10     // Period 10ms
#define MAX_DUTY 0.21     // Period 10ms
#define INITIAL_DUTY 0.1  // Esc is initialized by 0.1 duty

class Esc : public PwmOut {
public:
  Esc(PinName pwm_pin);
  ~Esc();
  void update(float input);
  void stop();

private:
  void _write(float valid_duty);
};