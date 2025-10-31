#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#define BMI088_USE_SPI

#include <stdint.h>

#define INT_GYRO 2
#define INT_ACCEL 1
#define CS_ACCEL 9
#define CS_GYRO 10
#define IMU_CLK 12
#define IMU_MOSI 11
#define IMU_MISO 13
#define HEATER_PWM 14

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif
