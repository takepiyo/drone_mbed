#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

class PIDController {
public:
  struct Param {
    float P;
    float I;
    float D;
  };

  // _Ts: Control loop period
  // _param: Controller parameter
  PIDController(float _Ts, const Param &_param) : Ts(_Ts), param(_param) {
    reset();
  }

  virtual ~PIDController() {}

  float update(float r, float y) {
    float error = r - y;
    integral_state += error;

    float u   = param.P * error + param.I * integral_state + param.D * (error - pre_error) / Ts;
    pre_error = error;

    return u;
  }

  void reset() {
    integral_state = 0.0;
    pre_error      = 0.0;
  }

private:
  const float Ts;
  const Param param;
  float integral_state;
  float pre_error;
};

#endif /* PIDCONTROLLER_H_ */