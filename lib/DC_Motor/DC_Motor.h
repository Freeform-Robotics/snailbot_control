/* DC motor library for Arduino
created by Ashutosh M Bhatt (+91-9998476150) - Gujarat (INDIA) 
ver 1.0 - 12/10/2016

this library is used to control DC motors
it can control its speed from 0 to 100% and its direction
it can rotate motor forward and reverse at set speed 
it can start or stop the motor as well as provides DC BREAK for instant STOP 
just one has to select arduino pins for motor,then then start rotating motor using given library functions
*/
#ifndef DC_Motor_h
#define DC_Motor_h

#include "Arduino.h"
#include "driver/mcpwm.h"

class DC_Motor
  { 
    private:
      int pin1_, pin2_;//, motor_speed;
      mcpwm_unit_t mcpwm_num_;
      mcpwm_timer_t timer_num_;
      float last_speed = 0.;
      unsigned long last_speed_time;
      float target_speed = 0.;
      float wheel_acc;
    public:
    DC_Motor(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, int pin1, int pin2, float wheel_acceleration=6.);
    float get_speed(void) {return last_speed;}
    float get_target_speed(void) {return target_speed;}
    
    void speed(float speed); 
    float loop(void);
    
    void stop(void);
    void brake(void); 

  };
  
#endif
