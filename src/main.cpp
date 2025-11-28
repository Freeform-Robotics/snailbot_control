#include <Arduino.h>
#include "main.h"
#include "gpio_config.h"
#include "crc.h"
#include "DC_Motor.h"
#include "Differential_Driver.h"
#include "BMI088driver.h"
#include "serial.h"
#include "imu.h"

#include <driver/adc.h> 
#include <driver/mcpwm.h> 
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

void uart_task(void);
void motor_test(void);
void setup_current_sensing(); 

// --- 全局对象 ---
HardwareSerial toRK3588Serial(1);

hw_timer_t *timer = NULL, *timer_slow = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerSlowMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool timerFlag = false;
volatile bool timerSlowFlag = false;

#define ADC_CHAN_MOTOR_A ADC1_CHANNEL_5 
#define ADC_CHAN_MOTOR_B ADC1_CHANNEL_2


volatile int acc_adc_motor_A = 0;
volatile int acc_adc_motor_B = 0;
volatile int sample_count_A = 0;
volatile int sample_count_B = 0;


volatile int raw_adc_motor_A = 0;
volatile int raw_adc_motor_B = 0;

int voltage_motor_A = 0;
int voltage_motor_B = 0;

volatile unsigned long isr_trigger_count = 0;

control_data_t control_data;
rc_data_t rc_data;
wheel_delta_odom_t wheel_delta_odom;

DC_Motor motor_A(MCPWM_UNIT_1, MCPWM_TIMER_2, AIN1, AIN2);
DC_Motor motor_B(MCPWM_UNIT_0, MCPWM_TIMER_2, BIN1, BIN2);
ESP32Encoder encoder_A;
ESP32Encoder encoder_B;
DifferentialDriver base_driver(&motor_A, &encoder_A, &motor_B, &encoder_B, 0);


static void IRAM_ATTR mcpwm_isr_handler(void *arg) {
    uint32_t status_0 = MCPWM0.int_st.val;
    uint32_t status_1 = MCPWM1.int_st.val;

    bool triggered = false;

    if ((status_1 & MCPWM_TIMER2_TEZ_INT_ENA) || (status_1 & MCPWM_TIMER2_TEP_INT_ENA)) {
        int val = 0;
        val = adc1_get_raw(ADC_CHAN_MOTOR_A);
        
        acc_adc_motor_A += val;
        sample_count_A++;
        if (sample_count_A >= 2) {
            int current_avg = acc_adc_motor_A >> 1; 
            raw_adc_motor_A = (raw_adc_motor_A * 9 + current_avg) / 10;

            acc_adc_motor_A = 0;
            sample_count_A = 0;
        }

        MCPWM1.int_clr.timer2_tez_int_clr = 1;
        MCPWM1.int_clr.timer2_tep_int_clr = 1;
        triggered = true;
    }

    if ((status_0 & MCPWM_TIMER2_TEZ_INT_ENA) || (status_0 & MCPWM_TIMER2_TEP_INT_ENA)) {
        int val = 0;
        val = adc1_get_raw(ADC_CHAN_MOTOR_B);
        
        acc_adc_motor_B += val;
        sample_count_B++;

        if (sample_count_B >= 2) {
            int current_avg = acc_adc_motor_B >> 1; // 除以2
            raw_adc_motor_B = (raw_adc_motor_B * 9 + current_avg) / 10;
            acc_adc_motor_B = 0;
            sample_count_B = 0;
        }

        MCPWM0.int_clr.timer2_tez_int_clr = 1;
        MCPWM0.int_clr.timer2_tep_int_clr = 1;
        triggered = true;
    }

    if (triggered) {
        isr_trigger_count++;
    }
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  timerFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR onTimerSlow() {
  portENTER_CRITICAL_ISR(&timerSlowMux);
  timerSlowFlag = true;
  portEXIT_CRITICAL_ISR(&timerSlowMux);
}

void setup() {
  // Debug Serial
  Serial.begin(115200);
  while(!Serial)
    delay(100);
  Serial.println("Debug serial started");

  // RK3588 Serial
  toRK3588Serial.begin(230400, SERIAL_8N1, RK3588_RX, RK3588_TX);
  while(!toRK3588Serial)
    delay(100);
  Serial.println("RK3588 serial started");

  // Initialize IMU
  imu_init();

  // Initialize Base Driver
  base_driver.initialize();
  base_driver.set_speed_pid(2.0, 0.0, 0.0);
  base_driver.enable_velocity_control();

  // Initialize Serial Packet
  control_data.vel_x = 0.0;
  control_data.vel_rot = 0.0;
  wheel_delta_odom.header = 0x04;

  setup_current_sensing();
  Serial.println("Dual-Point Current Sensing Initialized");
  timer = timerBegin(0, 640, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, SYSTICK, true);
  timerAlarmEnable(timer);

  // // Initialize Timer 125Hz
  // timer = timerBegin(0, 640, true);
  // timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, SYSTICK, true);
  // timerAlarmEnable(timer);

  // // Initialize Timer 0.5Hz
  // timer_slow = timerBegin(1, 40000, true);
  // timerAttachInterrupt(timer_slow, &onTimerSlow, true);
  // timerAlarmWrite(timer_slow, 4000, true);
  // timerAlarmEnable(timer_slow);

  Serial.println("Setup completed");
}

void setup_current_sensing() {
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC_CHAN_MOTOR_A, ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ADC_CHAN_MOTOR_B, ADC_ATTEN_DB_0);

    MCPWM1.int_ena.timer2_tez_int_ena = 1; 
    MCPWM1.int_ena.timer2_tep_int_ena = 1; 
    
    MCPWM0.int_ena.timer2_tez_int_ena = 1; 
    MCPWM0.int_ena.timer2_tep_int_ena = 1; 
    
    mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    mcpwm_isr_register(MCPWM_UNIT_1, mcpwm_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);

    Serial.println("Current Sensing ISR registered via mcpwm_isr_register");
}

void loop() {
  // on_timer_125hz_task();
  // on_timer_0_5hz_task();
  accel_callback();
  gyro_callback();
  serial_send_task();
  uart_task();
  motor_test();
  base_driver.loop();
}

void uart_task(void) {
  const size_t PACKET_SIZE_CONTROL = sizeof(control_data_t);
  const size_t PACKET_SIZE_RC = sizeof(rc_data_t);

  while (toRK3588Serial.available() > 0) {
    
    uint8_t header = toRK3588Serial.peek();

    if (header == 0x03) {
      if (toRK3588Serial.available() < PACKET_SIZE_CONTROL) return;

      uint8_t buf[PACKET_SIZE_CONTROL];
      toRK3588Serial.readBytes(buf, PACKET_SIZE_CONTROL);

      control_data_t temp_data;
      memcpy(&temp_data, buf, sizeof(temp_data));
      uint16_t correct_checksum = crc16((uint8_t*)&temp_data, sizeof(temp_data) - sizeof(temp_data.checksum));

      if (temp_data.checksum == correct_checksum) {
        // Serial.println("CMD"); 
        base_driver.speed_rotation_first(-temp_data.vel_x, temp_data.vel_rot);
      } else {
        Serial.println("CRC Error CMD");
      }
    }
    
    else if (header == 0xAA) {
      if (toRK3588Serial.available() < PACKET_SIZE_RC) return;
      
      uint8_t buf[PACKET_SIZE_RC];
      toRK3588Serial.readBytes(buf, PACKET_SIZE_RC);

      if (buf[3] == 0xBB) {
         memcpy(&rc_data, buf, sizeof(rc_data));
         double x = ((double)rc_data.x - 127.5) / 127.5;
         double y = ((double)rc_data.y - 127.5) / 127.5;
         if (abs(x) < 0.1) x = 0.0;
         if (abs(y) < 0.1) y = 0.0;
         base_driver.speed_rotation_first(y * 1.0, x * 1.0);
      }
    }
    else {
      toRK3588Serial.read(); 
    }
  }
}

void motor_test(void) {
    static unsigned long last_print_time = 0;

    if (millis() - last_print_time > 1000) {
        Serial.print("ISR Count: ");
        Serial.print(isr_trigger_count);
        
        voltage_motor_A = raw_adc_motor_A * 1100 / 4095;
        voltage_motor_B = raw_adc_motor_B * 1100 / 4095;
        Serial.printf(" | Volt: %d mV, %d mV\n", voltage_motor_A, voltage_motor_B);
        
        isr_trigger_count = 0;
        last_print_time = millis();
    }
    double vel_x = 0.2;     
    double vel_rot = 0.0; 
    base_driver.speed_rotation_first(vel_x, vel_rot);
}

void send_odom() {
  wheel_delta_odom.x = base_driver.get_delta_x();
  wheel_delta_odom.rot = base_driver.get_delta_rot();
  wheel_delta_odom.checksum = crc16((uint8_t*)&wheel_delta_odom, sizeof(wheel_delta_odom) - sizeof(wheel_delta_odom.checksum));
  toRK3588Serial.write((uint8_t*)&wheel_delta_odom, sizeof(wheel_delta_odom));
}

void on_timer_125hz_task() {
  if (timerFlag) {
    portENTER_CRITICAL(&timerMux);
    timerFlag = false;
    portEXIT_CRITICAL(&timerMux);
    
    // send_odom();
    // imu_filter_task();
  }
}

void on_timer_0_5hz_task() {
  if (timerSlowFlag) {
    portENTER_CRITICAL(&timerSlowMux);
    timerSlowFlag = false;
    portEXIT_CRITICAL(&timerSlowMux);
    
    temp_task();
  }
}