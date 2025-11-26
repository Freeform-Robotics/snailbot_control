#include <Arduino.h>
#include "main.h"
#include "gpio_config.h"
#include "crc.h"
#include "DC_Motor.h"
#include "Differential_Driver.h"
#include "BMI088driver.h"
#include "serial.h"
#include "imu.h"

void uart_callback(void);
void uart_task(void);
void motor_test(void);
HardwareSerial toRK3588Serial(1);

hw_timer_t *timer = NULL, *timer_slow = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerSlowMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool timerFlag = false;
volatile bool timerSlowFlag = false;

portMUX_TYPE uartMux = portMUX_INITIALIZER_UNLOCKED;
bool toRK3588Serial_available = false;

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

control_data_t control_data;
rc_data_t rc_data;
wheel_delta_odom_t wheel_delta_odom;

DC_Motor motor_A(MCPWM_UNIT_1, MCPWM_TIMER_2, AIN1, AIN2);
DC_Motor motor_B(MCPWM_UNIT_0, MCPWM_TIMER_2, BIN1, BIN2);
ESP32Encoder encoder_A;
ESP32Encoder encoder_B;
DifferentialDriver base_driver(&motor_A, &encoder_A, &motor_B, &encoder_B, 0);

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
  toRK3588Serial.onReceive(uart_callback);
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

  // Initialize Timer 125Hz
  timer = timerBegin(0, 640, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, SYSTICK, true);
  timerAlarmEnable(timer);

  // Initialize Timer 0.5Hz
  timer_slow = timerBegin(1, 40000, true);
  timerAttachInterrupt(timer_slow, &onTimerSlow, true);
  timerAlarmWrite(timer_slow, 4000, true);
  timerAlarmEnable(timer_slow);

  Serial.println("Setup completed");
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

void loop() {
  on_timer_125hz_task();
  on_timer_0_5hz_task();
  accel_callback();
  gyro_callback();
  serial_send_task();
  uart_task();
  // motor_test();
  base_driver.loop();
}

void uart_callback(void) {
  portENTER_CRITICAL(&uartMux);
  toRK3588Serial_available = true;
  portEXIT_CRITICAL(&uartMux);
}

void uart_task(void) {
  portENTER_CRITICAL(&uartMux);
  if (!toRK3588Serial_available) {
    portEXIT_CRITICAL(&uartMux);
    return;
  }
  toRK3588Serial_available = false;
  portEXIT_CRITICAL(&uartMux);

  uint8_t buf[255];
  int len = toRK3588Serial.available();
  if (len > 255) len = 255;
  for(int i = 0; i < len; i++) {
    buf[i] = toRK3588Serial.read();
  }
  if (buf[0] == 0x03) {
    memcpy(&control_data, buf, sizeof(control_data));
    uint16_t correct_checksum = crc16((uint8_t*)&control_data, sizeof(control_data) - sizeof(control_data.checksum));
    if(control_data.checksum != correct_checksum) {
      Serial.printf("Invalid checksum %d, expect %d.\n", control_data.checksum, correct_checksum);
      return;
    }
    Serial.println("Serial Control: " + String(control_data.vel_x) + ", " + String(control_data.vel_rot));
    base_driver.speed_rotation_first(control_data.vel_x, control_data.vel_rot);
  }
  else if (buf[0] == 0xAA && buf[3] == 0xBB) {
    memcpy(&rc_data, buf, sizeof(rc_data));
    // Map rc_data.x and rc_data.y from [0, 255] to [-1.0, 1.0]
    double x = ((double)rc_data.x - 127.5) / 127.5;
    double y = ((double)rc_data.y - 127.5) / 127.5;
    // Deadzone
    if (abs(x) < 0.1) x = 0.0;
    if (abs(y) < 0.1) y = 0.0;
    // Map to velocity
    double vel_x = y * 1.0;      // Max speed 1.0 m/s
    double vel_rot = x * 1.0;   // Max rotation speed 1.0 rad/s
    base_driver.speed_rotation_first(vel_x, vel_rot);
    Serial.println("RC Control: " + String(vel_x) + ", " + String(vel_rot));
  }
  else {
    Serial.println("Invalid header: " + String(buf[0]));
  }
}

void motor_test(void) {
    static unsigned long last_print_time = 0;
    
    if (millis() - last_print_time > 1000) {
        Serial.println("Enter motor test");
        last_print_time = millis();
    }

    double vel_x = 0.2;     
    double vel_rot = 0.0; 
    base_driver.speed_rotation_first(vel_x, vel_rot);
}
