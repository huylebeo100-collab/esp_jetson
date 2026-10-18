#include "wheel.h"

static void pwmInit(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
#endif
}

static void pwmWrite(uint8_t pin, uint8_t ch, int duty) {
  duty = constrain(duty, 0, PWM_MAX);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

void Wheel::begin() {
  pinMode(_enL, OUTPUT);
  pinMode(_enR, OUTPUT);
  enableDriver(false);

  pwmInit(_pwmL, _chL);
  pwmInit(_pwmR, _chR);
  applyPWM(0);

  _enc.attachFullQuad(_encA, _encB);
  _enc.clearCount();
  _lastCount = 0;

  enableDriver(true);
}

void Wheel::enableDriver(bool on) {
  digitalWrite(_enL, on ? HIGH : LOW);
  digitalWrite(_enR, on ? HIGH : LOW);
}

void Wheel::applyPWM(int d) {
  d = constrain(d * _dir, -PWM_MAX, PWM_MAX);
  if (d >= 0) { pwmWrite(_pwmL, _chL, 0);  pwmWrite(_pwmR, _chR, d); }
  else        { pwmWrite(_pwmR, _chR, 0);  pwmWrite(_pwmL, _chL, -d); }
}

void Wheel::setPWM(int duty) {
  out = constrain(duty, -PWM_MAX, PWM_MAX);
  applyPWM(out);
}

void Wheel::resetState() {
  _I = 0;
  _prevErr = 0;
  meas = 0;
  _warmup = 3;
}

void Wheel::syncEncoder() {
  _lastCount = count();
}

int Wheel::pidCompute() {
  float err = target - meas;
  float P = Kp * err;

  if (_warmup > 0) _warmup--;              // bo qua tich phan vai nhip dau
  else             _I += Ki * err * DT;

  _I = constrain(_I, -(float)PWM_MAX, (float)PWM_MAX);   // chong windup

  float D = Kd * (err - _prevErr) / DT;
  _prevErr = err;

  float u = P + _I + D;

  // bu deadband
  if (fabsf(target) > 1.0f) {
    if      (u >  1) u += deadband;
    else if (u < -1) u -= deadband;
  }
  u = constrain(u, -(float)PWM_MAX, (float)PWM_MAX);

  if (fabsf(target) < 1.0f && fabsf(meas) < 5.0f) { _I = 0; u = 0; }
  return (int)u;
}

void Wheel::update(bool pidOn) {
  int64_t c = count();
  float raw = (float)(c - _lastCount) * K_RPM;
  _lastCount = c;

  meas = (1.0f - ALPHA) * meas + ALPHA * raw;   // loc thong thap

  if (pidOn) {
    out = pidCompute();
    applyPWM(out);
  }
}