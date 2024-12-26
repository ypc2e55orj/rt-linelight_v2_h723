#ifndef DATA_PID_H_
#define DATA_PID_H_

/* C++ */
#include <array>

class Pid {
 public:
  using Gain = std::array<float, 3>;

  void Reset(const Gain &gain) {
    gainKp_ = gain[0];
    gainKi_ = gain[1];
    gainKd_ = gain[2];
    Reset();
  }
  void Reset() {
    prevError_ = 0.0f;
    sumError_ = 0.0f;
  }

  float Get() { return p_ + i_ + d_; }
  float GetProportional() { return p_; }
  float GetIntegral() { return i_; }
  float GetDerivative() { return d_; }

  float Update(float target, float current, float t) {
    float error = target - current;
    sumError_ += (error + prevError_) * t / 2.0f;
    p_ = gainKp_ * error;
    i_ = gainKi_ * sumError_;
    d_ = gainKd_ * (error - prevError_) / t;
    prevError_ = error;
    return p_ + i_ + d_;
  }

 private:
  float gainKp_{};
  float gainKi_{};
  float gainKd_{};

  float p_{};
  float i_{};
  float d_{};

  float prevError_{};
  float sumError_{};
};

#endif  // DATA_PID_H_
