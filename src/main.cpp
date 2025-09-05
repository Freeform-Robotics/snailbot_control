#include <Arduino.h>
#include <stdint.h>
#include "gpio_config.h"
#include "imu.h"
#include "crc.h"
#include "DC_Motor.h"
#include "Differential_Driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// void IRAM_ATTR task(void);
void control_callback(void);
void control_loop(void);

HardwareSerial toRK3588Serial(1);

typedef struct {
  uint8_t header;
  double vel_x;
  double vel_rot;
  uint16_t checksum;
} control_data_t;

control_data_t control_data;

DC_Motor motor_A(MCPWM_UNIT_1, MCPWM_TIMER_2, AIN1, AIN2);
DC_Motor motor_B(MCPWM_UNIT_0, MCPWM_TIMER_2, BIN1, BIN2);
ESP32Encoder encoder_A;
ESP32Encoder encoder_B;
DifferentialDriver base_driver(&motor_A, &encoder_A, &motor_B, &encoder_B, 0);

void setup() {
  // Debug Serial
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Debug serial started");

  // RK3588 Serial
  toRK3588Serial.begin(115200, SERIAL_8N1, RK3588_RX, RK3588_TX);
  while(!toRK3588Serial);
  toRK3588Serial.onReceive(control_callback);
  // Serial.onReceive(control_callback);
  Serial.println("RK3588 serial started");
  toRK3588Serial.println("RK3588 serial started");

  // Initialize IMU
  // imu_init();

  // Initialize Base Driver
  base_driver.initialize();
  base_driver.set_speed_pid(2.0, 0.0, 0.0);
  base_driver.enable_velocity_control();

  // Initialize Serial Packet
  control_data.vel_x = 1.0;
  control_data.vel_rot = 0.0;

  // Initialize Control Loop
  xTaskCreatePinnedToCore(control_loop, "ControlLoop", 4096, NULL, 24, NULL, 0);

  Serial.println("Setup completed");
}

void loop() {
  delay(1000);
}

void control_loop(void *pvParameters) {
  while(1) {
    Serial.println("vel_x: " + String(control_data.vel_x) + ", vel_rot: " + String(control_data.vel_rot));
    base_driver.loop();
    vTaskDelay(1);
  }
}

void control_callback(void) {
  uint8_t buf[255];
  for(uint8_t i = 0; toRK3588Serial.available(); i++) {
    buf[i] = toRK3588Serial.read();
  }
  memcpy(&control_data, buf, sizeof(control_data));
  if(control_data.header != 0x03) {
    Serial.printf("Invalid header %d, expect 0x03.\n", control_data.header);
    return;
  }
  uint16_t correct_checksum = crc16((uint8_t*)&control_data, sizeof(control_data) - 2);
  if(control_data.checksum != correct_checksum) {
    Serial.printf("Invalid checksum %d, expect %d.\n", control_data.checksum, correct_checksum);
    return;
  }
  base_driver.speed_rotation_first(control_data.vel_x, control_data.vel_rot);
}
