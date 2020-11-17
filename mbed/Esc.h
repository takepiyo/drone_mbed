#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32.h>

#define PERIOD  10
#define MIN_DUTY 0.10  //Period 10ms
#define MAX_DUTY 0.21  //Period 10ms
#define INITIAL_DUTY 0.1  //Esc is initialized by 0.1 duty

class Esc
{
  public:
  Esc(PinName pwm_pin);
  // Esc(PinName pwm_pin, ros::NodeHandle nh);
  ~Esc();
  void update(std_msgs::Float32 input);
  void stop();

  private:
  bool _is_debug;
  PwmOut _motor(PinName pwm_pin);
  std_msgs::Float32 _duty
  ros::Publisher _debugger("debug_message", &_duty); 
  void _write(float valid_duty);
}