#include "imu.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "gpio_config.h"
#include "BMI088.h"
#include "pid.h"
#include "crc.h"

#define TEMPERATURE_DESIRED 45.0f
#define TEMPERATURE_PID_KP 1600.0f         // kp of temperature control PID
#define TEMPERATURE_PID_KI 0.2f            // ki of temperature control PID
#define TEMPERATURE_PID_KD 0.0f            // kd of temperature control PID
#define TEMPERATURE_PID_MAX_OUT 4998.0f    // max out of temperature control PID
#define TEMPERATURE_PID_MAX_I_OUT 4400.0f  // max I out of temperature control PID

Bmi088 bmi(SPI, CS_ACCEL, CS_GYRO);
pid_type_def temp_pid;

accel_data_t accel_data;
gyro_data_t gyro_data;

extern HardwareSerial toRK3588Serial;

void heater_init() {    // Active high heater pwm
    // Initialize MCPWM unit 0, timer 0, on GPIO 18
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, HEATER_PWM);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 14400;   // PWM frequency = 14.4kHz
    pwm_config.cmpr_a = 0.0;        // Duty cycle of PWMxA = 0.0%
    pwm_config.cmpr_b = 0.0;        // Duty cycle of PWMxB = 0.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Initialize PID
    float pid_params[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
    PID_init(&temp_pid, PID_POSITION, pid_params, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_I_OUT);
}

void sensor_init() {
    int status;

    /* start the sensors */
    status = bmi.begin();
    if (status < 0) {
        Serial.println("IMU Initialization Error");
        Serial.println(status);
        while (1) {}
    }
    /* set the ranges */
    status = bmi.setRange(Bmi088::ACCEL_RANGE_3G,Bmi088::GYRO_RANGE_125DPS);
    if (status < 0) {
        Serial.println("Failed to set ranges");
        Serial.println(status);
        while (1) {}
    }
    /* set the output data rate */
    status = bmi.setOdr(Bmi088::ODR_2000HZ);
    if (status < 0) {
        Serial.println("Failed to set ODR");
        Serial.println(status);
        while (1) {}
    }
    /* specify whether to use pin 3 or pin 4 to loop back the gyro interrupt */
    status = bmi.mapSync(Bmi088::PIN_3);
    if (status < 0) {
        Serial.println("Failed to map sync pin");
        Serial.println(status);
        while (1) {}
    }
    /* 
    * specify whether to use pin 1 or pin 2 to indicate data ready, the other pin will be used
    * for gyro interrupt input 
    */
    status = bmi.mapDrdy(Bmi088::PIN_1);
    if (status < 0) {
        Serial.println("Failed to map data ready pin");
        Serial.println(status);
        while (1) {}
    }
    /* set the data ready pin to push-pull and active high */
    status = bmi.pinModeDrdy(Bmi088::OUT_PP,Bmi088::ACTIVE_HIGH);
    if (status < 0) {
        Serial.println("Failed to setup data ready pin");
        Serial.println(status);
        while (1) {}
    }
    /* attach the corresponding uC pin to an interrupt */
    pinMode(INT_GYRO,INPUT);
    attachInterrupt(INT_GYRO, gyro_callback, RISING);
    pinMode(INT_ACCEL,INPUT);
    attachInterrupt(INT_ACCEL, accel_callback, RISING);
}

void imu_init() {
    heater_init();
    sensor_init();
    accel_data.header = 0x01;
    gyro_data.header = 0x02;
}

void temp_task(void) {
    PID_calc(&temp_pid, bmi.getTemperature_C(), TEMPERATURE_DESIRED);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, temp_pid.out);
}

void gyro_callback(void) {
    bmi.readSensor();
    Serial.println(bmi.getGyroX_rads());
    Serial.println(bmi.getGyroY_rads());
    Serial.println(bmi.getGyroZ_rads());
    gyro_data.gyro_x = bmi.getGyroX_rads();
    gyro_data.gyro_y = bmi.getGyroY_rads();
    gyro_data.gyro_z = bmi.getGyroZ_rads();
    gyro_data.checksum = crc16((uint8_t*)&gyro_data, sizeof(gyro_data) - 2);
    toRK3588Serial.write((uint8_t*)&gyro_data, sizeof(gyro_data));
}

void accel_callback(void) {
    bmi.readSensor();
    Serial.println(bmi.getAccelX_mss());
    Serial.println(bmi.getAccelY_mss());
    Serial.println(bmi.getAccelZ_mss());
    accel_data.accel_x = bmi.getAccelX_mss();
    accel_data.accel_y = bmi.getAccelY_mss();
    accel_data.accel_z = bmi.getAccelZ_mss();
    accel_data.checksum = crc16((uint8_t*)&accel_data, sizeof(accel_data) - 2);
    toRK3588Serial.write((uint8_t*)&accel_data, sizeof(accel_data));
}
