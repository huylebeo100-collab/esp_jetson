#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config.h"

class Wheel {
public:
  Wheel(uint8_t enL, uint8_t enR, uint8_t pwmL, uint8_t pwmR,
        uint8_t chL, uint8_t chR, uint8_t encA, uint8_t encB,
        int8_t dirSign, int8_t encSign,
        float kp, float ki, float kd, int db)
    : Kp(kp), Ki(ki), Kd(kd), deadband(db),
      _enL(enL), _enR(enR), _pwmL(pwmL), _pwmR(pwmR),
      _chL(chL), _chR(chR), _encA(encA), _encB(encB),
      _dir(dirSign), _encDir(encSign) {}

  void begin();
  void enableDriver(bool on);

  // dieu khien tho (PID phai TAT)
  void setPWM(int duty);

  // dat lenh
  void setTargetRpm(float rpm) { target = constrain(rpm, -MAX_RPM, MAX_RPM); }
  void setTargetMps(float mps) { setTargetRpm(mps * MPS_TO_RPM); }

  // goi dung 50 Hz
  void update(bool pidOn);

  void resetState();      // xoa I, D, bo loc, warmup
  void syncEncoder();     // dong bo lai sau lenh chan lau (delay)
  void clearCount()  { _enc.clearCount(); _lastCount = 0; }

  // KHONG the la const: ESP32Encoder::getCount() khong phai ham const
  int64_t count() { return (int64_t)_encDir * _enc.getCount(); }
  float   rpm() const { return meas; }
  float   mps() const { return meas * RPM_TO_MPS; }

  // tham so mo (de chinh nong qua Serial)
  float Kp, Ki, Kd;
  int   deadband;
  float target = 0;    // rpm
  float meas   = 0;    // rpm da loc
  int   out    = 0;    // pwm cuoi cung (-1023..1023)

private:
  int  pidCompute();
  void applyPWM(int duty);   // duty da tinh ca dau

  uint8_t _enL, _enR, _pwmL, _pwmR, _chL, _chR, _encA, _encB;
  int8_t  _dir, _encDir;
  ESP32Encoder _enc;
  int64_t _lastCount = 0;
  float _I = 0, _prevErr = 0;
  int   _warmup = 0;
};