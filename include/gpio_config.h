#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

// Motor Control
#define MOTOR_NSLEEP 6
#define AIN1 17
#define AIN2 16
#define BIN1 15
#define BIN2 7

// Encoder
#define AWE1 18
#define AWE2 8
#define BWE1 4
#define BWE2 5

// BMI088 via SPI
#define INT_GYRO 13
#define INT_ACCEL 14
#define CS_ACCEL 26
#define CS_GYRO 29
#define IMU_CLK 30
#define IMU_MOSI 31
#define IMU_MISO 32
#define HEATER_PWM 12

// UART to RK3588
#define RK3588_RX 19
#define RK3588_TX 20

// Debug UART
#define DEBUG_TX 43
#define DEBUG_RX 44

#endif
