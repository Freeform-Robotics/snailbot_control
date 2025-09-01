
#ifndef Differential_Driver_h
#define Differential_Driver_h

#include "Arduino.h"
#include "DC_Motor.h"
#include <ESP32Encoder.h>
#include "PID_v1.h"

#define INVALID_SPEED_AMP 0.08
#define POSITION_DEAD_ZONE 0.01

#define BASE_MOTOR_INVALID_AMP 0.1

#define MOTOR_RATIO_RAD (3.f * 2.f / 2.5f * PI / 7000.f / 1.6f)
#define GEAR_RATIO 1600
#define WHEEL_RADIUS 0.02
#define WHEEL_DISTANCE 0.08075
#define INSPECT_OK_CODE (49 << 2)

class DifferentialDriver
{ 
  private:
    DC_Motor *motor_l, *motor_r;
    ESP32Encoder *encoder_l, *encoder_r;
    uint32_t eeprom_offset;
    
    int64_t encoder_l_last_count, encoder_r_last_count;
    uint32_t timer;

    PID velocity_l_pid, velocity_r_pid;
    PID orientation_pid, position_pid;


    double orientation_error, expected_angular_speed, orientation_setpoint = 0.; // orientation_setpoint is always zero
    double position_error, expected_linear_speed, position_setpoint = 0.; // position_setpoint is always zero

    bool motor_l_inverted = false, motor_r_inverted = false;
    
  public:
    DifferentialDriver(DC_Motor*, ESP32Encoder*, DC_Motor*, ESP32Encoder*, uint32_t);
    
    void speed_rotation_first(void) { speed_rotation_first(expected_linear_speed, expected_angular_speed); } 
    void speed_rotation_first(double, double); 
    void speed_linear_leading(double, double);
    void speed_raw(double, double);
    void pwm(float, float);
    bool check_stop(void);
    
    void loop(void);
    void set_pose_error(float, float);
    void set_pose_error(float, float, float*, float*);

    void enable_velocity_control(void);
    void brake(void) { motor_l->brake(); motor_r->brake(); }

    void set_speed_pid(double, double, double);
    void set_orientation_pid(double, double, double);
    void set_position_pid(double, double, double);

    void initialize(void);
    uint8_t inspect(void);

    size_t copy_debug_data(byte* addr);
    double motor_l_real_speed, motor_l_pwm, motor_l_target_speed = 0.;
    double motor_r_real_speed, motor_r_pwm, motor_r_target_speed = 0.;

    double last_l_pwm, last_r_pwm;
};
  
#endif
