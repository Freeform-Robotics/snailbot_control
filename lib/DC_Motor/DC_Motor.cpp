#include "Arduino.h"
#include "DC_Motor.h"
#include "driver/mcpwm.h"


DC_Motor::DC_Motor(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, int pin1, int pin2, float wheel_acceleration)   // this is a constructor
{
  mcpwm_num_ = mcpwm_num;
  timer_num_ = timer_num;
  pin1_ = pin1;
  pin2_ = pin2;
  wheel_acc = wheel_acceleration;
  last_speed_time = micros();

  mcpwm_io_signals_t io_signal_pin1, io_signal_pin2;
  switch(timer_num_){
    case MCPWM_TIMER_0:
      io_signal_pin1 = MCPWM0A;
      io_signal_pin2 = MCPWM0B;
      break; 
    case MCPWM_TIMER_1:
      io_signal_pin1 = MCPWM1A;
      io_signal_pin2 = MCPWM1B;
      break; 
    case MCPWM_TIMER_2:
      io_signal_pin1 = MCPWM2A;
      io_signal_pin2 = MCPWM2B;
      break; 
  }

  mcpwm_gpio_init(mcpwm_num, io_signal_pin1, pin1_);
  mcpwm_gpio_init(mcpwm_num, io_signal_pin2, pin2_);
  
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;    //frequency = 500Hz,
  pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(mcpwm_num_, timer_num_, &pwm_config);    //Configure PWM0A & PWM0B with above settings

  stop();
    
}
/////////////////////////// this function will start rotating motor in either direction //////////////////////////////  
 
//////////////////////////////////// this function will apply DC break to motor ///////////////////////////////////////  
void DC_Motor::brake()                        
{  
  mcpwm_set_signal_high(mcpwm_num_, timer_num_, MCPWM_OPR_A);
  mcpwm_set_signal_high(mcpwm_num_, timer_num_, MCPWM_OPR_B);
  last_speed = 0.;
  target_speed = 0.;                                           
} 
          
void DC_Motor::stop()                                                            
{
  mcpwm_set_signal_low(mcpwm_num_, timer_num_, MCPWM_OPR_A);
  mcpwm_set_signal_low(mcpwm_num_, timer_num_, MCPWM_OPR_B); 
} 
  
void DC_Motor::speed(float speed)                       
{
  if (speed > 1.) target_speed = 1.;
  else if (speed < -1.) target_speed = -1.;
  else target_speed = speed;
} 

float DC_Motor::loop(void)
{
  unsigned long speed_time = micros();
  unsigned long dt_us = (unsigned long)(speed_time - last_speed_time);
  float dt = ((float) dt_us) / 1.e6f;  // dt should never less than zero.
  if (abs(target_speed - last_speed) > wheel_acc * dt) {
    if (target_speed > last_speed) last_speed = last_speed + wheel_acc * dt;
    else last_speed = last_speed - wheel_acc * dt;
    if (last_speed > 1.) last_speed = 1.;
    if (last_speed < -1.) last_speed = -1.;
  }
  else { last_speed = target_speed; }
  last_speed_time = speed_time;

  float duty_cycle = last_speed * 100.;
  if (abs(duty_cycle) < 1e-3){
    stop();
  }
  else if (duty_cycle > 0){
    duty_cycle = duty_cycle > 100.0 ? 100.0 : duty_cycle;
    mcpwm_set_signal_low(mcpwm_num_, timer_num_, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num_, timer_num_, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num_, timer_num_, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
  }
  else {
    duty_cycle = duty_cycle < -100.0 ? 100.0 : -duty_cycle;
    mcpwm_set_signal_low(mcpwm_num_, timer_num_, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num_, timer_num_, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num_, timer_num_, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
  }  
  return last_speed;
}
