#include "BMI088Middleware.h"
#include "Arduino.h"
#include <SPI.h>

// Define SPI settings for BMI088
SPISettings bmi088SPISettings(1000000, MSBFIRST, SPI_MODE0); // 1MHz, MSB first, Mode 0

void BMI088_GPIO_init(void) {
    pinMode(CS_ACCEL, OUTPUT);
    pinMode(CS_GYRO, OUTPUT);
    digitalWrite(CS_ACCEL, HIGH);
    digitalWrite(CS_GYRO, HIGH);
}

void BMI088_com_init(void) {
    SPI.begin(IMU_CLK, IMU_MISO, IMU_MOSI);
    SPI.setFrequency(1000000); // Set SPI frequency to 1MHz
}

void BMI088_delay_ms(uint16_t ms) {
    delay(ms);
}

void BMI088_delay_us(uint16_t us) {
    delayMicroseconds(us);
}

void BMI088_ACCEL_NS_L(void)
{
    digitalWrite(CS_ACCEL, LOW);
}

void BMI088_ACCEL_NS_H(void)
{
    digitalWrite(CS_ACCEL, HIGH);
}

void BMI088_GYRO_NS_L(void)
{
    digitalWrite(CS_GYRO, LOW);
}

void BMI088_GYRO_NS_H(void)
{
    digitalWrite(CS_GYRO, HIGH);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    SPI.beginTransaction(bmi088SPISettings);
    uint8_t rx_data = SPI.transfer(txdata);
    SPI.endTransaction();
    return rx_data;
}