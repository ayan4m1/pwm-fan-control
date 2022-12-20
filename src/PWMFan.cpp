#include "PWMFan.hpp"

PWMFan::PWMFan(const uint8_t tach_pin, const uint8_t pwm_pin,
               const uint16_t min_rpm = 100, const uint8_t hz_to_rpm = 30) {
  this->tach_pin = tach_pin;
  this->pwm_pin = pwm_pin;
  this->min_rpm = min_rpm;
  this->hz_to_rpm = hz_to_rpm;
  this->tach_timeout = (1 / (min_rpm / hz_to_rpm)) * 1e6;
}

void PWMFan::begin() {
  pinMode(tach_pin, INPUT);
  pinMode(pwm_pin, OUTPUT);

  ledcSetup(FAN_LEDC_CHANNEL, FAN_PWM_FREQUENCY_HZ, FAN_PWM_RESOLUTION_BITS);
  ledcAttachPin(pwm_pin, FAN_LEDC_CHANNEL);
}

void PWMFan::end() { ledcDetachPin(pwm_pin); }

void PWMFan::update() {
  uint32_t risingPulseUs = pulseIn(tach_pin, LOW, tach_timeout);
  uint32_t fallingPulseUs = pulseIn(tach_pin, HIGH, tach_timeout);
  uint16_t totalPulseMs = (risingPulseUs + fallingPulseUs) / 1e3;

  if (totalPulseMs <= 0) {
    current_rpm = 0;
  } else {
    double speedHz = totalPulseMs > 0 ? (1e3 / totalPulseMs) : 0;
    current_rpm = speedHz * hz_to_rpm;
  }

  if (pids.Kp == 0) {
    return;
  }

  epid_pid_calc(&pid, target_rpm, current_rpm);
  epid_pid_sum(&pid, FAN_SPEED_MIN, FAN_SPEED_MAX);

  ledcWrite(FAN_LEDC_CHANNEL, (uint32_t)lroundf(pid.y_out));
}

void PWMFan::setDutyCycle(const uint8_t duty_cycle) {
  this->duty_cycle = duty_cycle;
}

void PWMFan::setTargetRpm(const uint16_t target_rpm) {
  this->target_rpm = target_rpm;
}

bool PWMFan::setPIDs(const PIDs pids) {
  this->pids.Kp = pids.Kp;
  this->pids.Ki = pids.Ki;
  this->pids.Kd = pids.Kd;

  auto epid_err = epid_init(&pid, current_rpm, current_rpm, FAN_SPEED_MIN,
                            pids.Kp, pids.Ki, pids.Kd);

  return epid_err == EPID_ERR_NONE;
}
