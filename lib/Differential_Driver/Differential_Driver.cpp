#include "Arduino.h"
#include "Differential_Driver.h"
#include <FunctionalInterrupt.h>
#include <EEPROM.h>

#define MAX_SPEED 2.25


DifferentialDriver::DifferentialDriver(DC_Motor* motor_l_, ESP32Encoder* encoder_l_, DC_Motor* motor_r_, ESP32Encoder* encoder_r_, uint32_t eeprom_offset_)
:velocity_l_pid(&motor_l_real_speed, &motor_l_pwm, &motor_l_target_speed, 0.2, 3., 0, DIRECT),
velocity_r_pid(&motor_r_real_speed, &motor_r_pwm, &motor_r_target_speed, 0.2, 3., 0, DIRECT),
orientation_pid(&orientation_error, &expected_angular_speed, &orientation_setpoint, 12., 0., 0, DIRECT),
position_pid(&position_error, &expected_linear_speed, &position_setpoint, 25., 0.2, 0, DIRECT)
{
  motor_l = motor_l_;
  encoder_l = encoder_l_;
  motor_r = motor_r_;
  encoder_r = encoder_r_;
  eeprom_offset = eeprom_offset_;
  //invert_pose = (base_id == 1); // invert_pose is true means base1

  velocity_l_pid.SetMode(AUTOMATIC);
  velocity_l_pid.SetOutputLimits(-1.0, 1.0);
  velocity_l_pid.SetSampleTime(5);
  velocity_r_pid.SetMode(AUTOMATIC);
  velocity_r_pid.SetOutputLimits(-1.0, 1.0);
  velocity_r_pid.SetSampleTime(5);

  orientation_pid.SetMode(AUTOMATIC);
  orientation_pid.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  orientation_pid.SetSampleTime(10);
  position_pid.SetMode(AUTOMATIC);
  position_pid.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  position_pid.SetSampleTime(10);

  timer = micros();
  encoder_l_last_count = 0;
  encoder_r_last_count = 0;

  orientation_setpoint = 0.;
  position_setpoint = 0.;
}

void DifferentialDriver::speed_rotation_first(double linear, double angular)                       
{
  if (linear + angular > MAX_SPEED) {
    linear = MAX_SPEED - angular;
  }
  else if (linear + angular < -MAX_SPEED) {
    linear = -MAX_SPEED - angular;
  }
  else if (linear - angular > MAX_SPEED) {
    linear = MAX_SPEED + angular;
  }
  else if (linear - angular < -MAX_SPEED) {
    linear = -MAX_SPEED + angular;
  }

  motor_l_target_speed = linear - angular;
  motor_r_target_speed = -(linear + angular);
}

void DifferentialDriver::speed_linear_leading(double linear, double angular)
{
  // not implement yet
  if (abs(orientation_error) > 30.f * PI / 180.) linear = 0.;

  if (linear + angular > MAX_SPEED) {
    linear = MAX_SPEED - angular;
  }
  else if (linear + angular < -MAX_SPEED) {
    linear = -MAX_SPEED - angular;
  }
  else if (linear - angular > MAX_SPEED) {
    linear = MAX_SPEED + angular;
  }
  else if (linear - angular < -MAX_SPEED) {
    linear = -MAX_SPEED + angular;
  }

  motor_l_target_speed = linear - angular;
  motor_r_target_speed = -(linear + angular);
}

void DifferentialDriver::speed_raw(double linear, double angular)
{
  // the input should be safe (less than MAX_SPEED), through unsafe is ok
  motor_l_target_speed = (linear + (angular * WHEEL_DISTANCE)/2);
  motor_r_target_speed = -(linear - (angular * WHEEL_DISTANCE)/2);
  // motor_l_target_speed = (-linear - (-angular * WHEEL_DISTANCE) / 2) * GEAR_RATIO / WHEEL_RADIUS / 1000;
  // motor_r_target_speed = (linear + (-angular * WHEEL_DISTANCE) / 2) * GEAR_RATIO / WHEEL_RADIUS / 1000;
}

// manually set pwm
void DifferentialDriver::pwm(float linear, float angular)
{
  if (velocity_l_pid.GetMode() != MANUAL) velocity_l_pid.SetMode(MANUAL);
  if (velocity_r_pid.GetMode() != MANUAL) velocity_r_pid.SetMode(MANUAL);
  motor_l_pwm = linear - angular;
  motor_r_pwm = -(linear + angular);
}

bool DifferentialDriver::check_stop(void)
{
  return (abs(motor_l->get_speed()) < BASE_MOTOR_INVALID_AMP) && (abs(motor_r->get_speed()) < BASE_MOTOR_INVALID_AMP);
}

void DifferentialDriver::loop(void)
{
  unsigned long dt_us = (unsigned long) (micros() - timer);
  double dt = (double)(dt_us) / 1000000.; // Calculate delta time
  
  // motor_l_real_speed = -((double) (encoder_l->getCount() - encoder_l_last_count)) * MOTOR_RATIO_RAD / dt;
  // motor_r_real_speed = -((double) (encoder_r->getCount() - encoder_r_last_count)) * MOTOR_RATIO_RAD / dt;
  Serial.println("loop");
  Serial.print(">motor_l_real_speed:");
  Serial.println(motor_l_real_speed);
  Serial.print(">motor_l_target_speed:");
  Serial.println(motor_l_target_speed);
  Serial.print(">motor_r_real_speed:");
  Serial.println(motor_r_real_speed);
  Serial.print(">motor_r_target_speed:");
  Serial.println(motor_r_target_speed);
  
  velocity_l_pid.Compute();
  velocity_r_pid.Compute();
  // Serial.print(">motor_l_pwm:");
  // Serial.println(motor_l_pwm);
  // Serial.print(">motor_r_pwm:");
  // Serial.println(motor_r_pwm);

  // motor_l_pwm += last_l_pwm;
  // motor_r_pwm += last_r_pwm;

  if (abs(motor_l_pwm) < BASE_MOTOR_INVALID_AMP) motor_l_pwm = 0.;
  if (abs(motor_r_pwm) < BASE_MOTOR_INVALID_AMP) motor_r_pwm = 0.;  

  // last_l_pwm = motor_l_pwm;
  // last_r_pwm = motor_r_pwm;

  if (motor_l_inverted) motor_l->speed(-motor_l_pwm);
  else motor_l->speed(motor_l_pwm);
  // else motor_l->speed(10.0);
  if (motor_r_inverted) motor_r->speed(-motor_r_pwm);
  else motor_r->speed(motor_r_pwm);
  // else motor_r->speed(10.0);
  motor_l->loop();
  motor_r->loop();
  
  timer = micros();
  // encoder_l_last_count = encoder_l->getCount();
  // encoder_r_last_count = encoder_r->getCount();
  
  //Serial.print(motor_l_real_speed, 4);Serial.print(' ');Serial.print(motor_l_pwm, 4);Serial.print(' ');Serial.print(motor_l_target_speed, 4);
  //Serial.print(' ');Serial.print(' ');Serial.print(' ');
  //Serial.print(motor_r_real_speed, 4);Serial.print(' ');Serial.print(motor_r_pwm, 4);Serial.print(' ');Serial.println(motor_r_target_speed, 4);
}

void DifferentialDriver::set_pose_error(float _position_error, float _orientation_error)
{
  position_error = _position_error;
  orientation_error = _orientation_error;

  orientation_setpoint = 0.;
  position_setpoint = 0.;

  orientation_pid.Compute();
  position_pid.Compute();
  // change expected_linear_speed and angular_speed_speed
}

void DifferentialDriver::set_pose_error(float _position_error, float _orientation_error, float* _expected_linear_speed, float* _angular_speed_speed)
{
  set_pose_error( _position_error, _orientation_error);
  *_expected_linear_speed = expected_linear_speed;
  *_angular_speed_speed = expected_angular_speed;
}

void DifferentialDriver::enable_velocity_control(void)
{
  if (velocity_l_pid.GetMode() != AUTOMATIC) velocity_l_pid.SetMode(AUTOMATIC);
  if (velocity_r_pid.GetMode() != AUTOMATIC) velocity_r_pid.SetMode(AUTOMATIC);
}

void DifferentialDriver::set_speed_pid(double Kp, double Ki, double Kd)
{
  velocity_l_pid.SetTunings(Kp, Ki, Kd);
  velocity_r_pid.SetTunings(Kp, Ki, Kd);
}

void DifferentialDriver::set_orientation_pid(double Kp, double Ki, double Kd)
{
  orientation_pid.SetTunings(Kp, Ki, Kd);
}

void DifferentialDriver::set_position_pid(double Kp, double Ki, double Kd)
{
  position_pid.SetTunings(Kp, Ki, Kd);
}

void DifferentialDriver::initialize(void)
{
  uint8_t motor_inspect_code = EEPROM.readByte(eeprom_offset);
  if ((motor_inspect_code & 0xFC) == INSPECT_OK_CODE) {
    motor_l_inverted = (bool) (motor_inspect_code & (1<<1));
    motor_r_inverted = (bool) (motor_inspect_code & 1);
  }
}

uint8_t DifferentialDriver::inspect(void)
{
  int64_t count_l = encoder_l->getCount();
  int64_t count_r = encoder_r->getCount();
  motor_l_inverted = false;
  motor_r_inverted = false;
  pwm(0, -1);
  vTaskDelay(100 / portTICK_RATE_MS);
  pwm(0, 0);
  count_l = encoder_l->getCount() - count_l; 
  count_r = encoder_r->getCount() - count_r;
  
  uint8_t err_code = 0;
  if (abs(count_l) == 0) err_code |= 1 << 1;
  if (abs(count_r) == 0) err_code |= 1;
  if (err_code != 0) return err_code;

  uint8_t motor_inspect_code = INSPECT_OK_CODE;
  if (count_l < 0) {
    motor_l_inverted = true;
    motor_inspect_code |= 1 << 1; // left motor inverted
  }
  if (count_r < 0) {
    motor_r_inverted = true;
    motor_inspect_code |= 1; // right motor inverted
  }

  EEPROM.writeByte(eeprom_offset, motor_inspect_code);
  EEPROM.commit();

  return err_code;
}

size_t DifferentialDriver::copy_debug_data(byte* addr)
{
  memcpy(addr, &motor_l_real_speed, sizeof(double));
  memcpy(addr + sizeof(double), &motor_l_pwm, sizeof(double));
  memcpy(addr + sizeof(double) * 2, &motor_l_target_speed, sizeof(double));
  memcpy(addr + sizeof(double) * 3, &motor_r_real_speed, sizeof(double));
  memcpy(addr + sizeof(double) * 4, &motor_r_pwm, sizeof(double));
  memcpy(addr + sizeof(double) * 5, &motor_r_target_speed, sizeof(double));
  memcpy(addr + sizeof(double) * 6, &orientation_error, sizeof(double));
  memcpy(addr + sizeof(double) * 7, &expected_angular_speed, sizeof(double));
  memcpy(addr + sizeof(double) * 8, &orientation_setpoint, sizeof(double));
  memcpy(addr + sizeof(double) * 9, &position_error, sizeof(double));
  memcpy(addr + sizeof(double) * 10, &expected_linear_speed, sizeof(double));
  memcpy(addr + sizeof(double) * 11, &position_setpoint, sizeof(double));
  return sizeof(double) * 12;
}
