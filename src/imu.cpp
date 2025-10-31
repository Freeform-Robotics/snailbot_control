#include "imu.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "gpio_config.h"
#include "pid.h"
#include "crc.h"
#include <SPI.h>
#include <Arduino.h>
#include "BMI088driver.h"
#include "serial.h"
#include <vector>
#include "MadgwickAHRS.h"

#define TEMPERATURE_DESIRED 45.0f
#define TEMPERATURE_PID_KP 15.0f         // kp of temperature control PID
#define TEMPERATURE_PID_KI 0.2f            // ki of temperature control PID
#define TEMPERATURE_PID_KD 0.0f            // kd of temperature control PID
#define TEMPERATURE_PID_MAX_OUT 100.0f    // max out of temperature control PID
#define TEMPERATURE_PID_MAX_I_OUT 100.0f  // max I out of temperature control PID

#define G 9.80665f

SPIClass SPI2(HSPI);
pid_type_def temp_pid;

Madgwick madgwick_filter;
imu_t imu_data;

float raw_accel[3];
float gravity_init[3] = {0.0f, 0.0f, 0.0f};

extern HardwareSerial toRK3588Serial;

volatile bool gyro_data_ready = false;
volatile bool accel_data_ready = false;
bool accel_data_updated = false;
bool gyro_data_updated = false;

uint64_t accel_timestamp = 0;
uint64_t gyro_timestamp = 0;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Quaternion of sensor frame relative to auxiliary frame

static inline float clampf(float v, float a, float b) {
    return (v < a) ? a : ( (v > b) ? b : v );
}

static void quaternion_normalize() {
    float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (n > 0.0f) {
        q0 /= n; q1 /= n; q2 /= n; q3 /= n;
    } else {
        q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    }
}

// Integrate body angular rates (rad/s) into quaternion (simple first-order integration)
static void integrate_gyro_quaternion(float gx, float gy, float gz, float dt) {
    // q_dot = 0.5 * q * [0, gx, gy, gz]
    float qw = q0, qx = q1, qy = q2, qz = q3;

    float qdot_w =  0.5f * (- qx * gx - qy * gy - qz * gz);
    float qdot_x =  0.5f * (  qw * gx + qy * gz - qz * gy);
    float qdot_y =  0.5f * (  qw * gy - qx * gz + qz * gx);
    float qdot_z =  0.5f * (  qw * gz + qx * gy - qy * gx);

    q0 += qdot_w * dt;
    q1 += qdot_x * dt;
    q2 += qdot_y * dt;
    q3 += qdot_z * dt;

    quaternion_normalize();
}

// Convert quaternion -> Euler (roll, pitch, yaw) in radians
static void quaternion_to_euler(float &roll, float &pitch, float &yaw) {
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = clampf(sinp, -1.0f, 1.0f);
    pitch = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2f(siny_cosp, cosy_cosp);
}

void IRAM_ATTR gyro_int_isr() {
    gyro_data_ready = true;
}

void IRAM_ATTR accel_int_isr() {
    accel_data_ready = true;
}

void rotate_vector_rpy(const float v_in[3], float roll, float pitch, float yaw, float v_out[3]) {
    // Rotation order: Z (yaw), Y (pitch), X (roll)
    float cr = cos(roll), sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw), sy = sin(yaw);

    // Rotation matrix (ZYX) mapping body -> world
    float R[3][3] = {
        {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
        {sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
        {-sp,   cp*sr,            cp*cr}
    };

    // Use R^T to map world/initial -> body/current (inverse rotation)
    for (int i = 0; i < 3; ++i) {
        v_out[i] = R[0][i]*v_in[0] + R[1][i]*v_in[1] + R[2][i]*v_in[2];
    }
}

void calibrate_gravity() {
    // Sample several times for better accuracy
    const int samples = 100;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < samples; ++i) {
        get_BMI088_accel(raw_accel);
        sum[0] += raw_accel[0];
        sum[1] += raw_accel[1];
        sum[2] += raw_accel[2];
        delay(10); // Wait for sensor to settle
    }
    gravity_init[0] = sum[0] / samples;
    gravity_init[1] = sum[1] / samples;
    gravity_init[2] = sum[2] / samples;
}

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
    uint8_t err = BMI088_init();
    if (err != BMI088_NO_ERROR) {
        Serial.print("BMI088 Initialization Error: ");
        Serial.println(err);
        while (1) {}
    }
    BMI088_set_accel_int_drdy(1);
    BMI088_set_gyro_int_drdy(3);
    calibrate_gravity();
}

void imu_init() {
    heater_init();
    sensor_init();
    attachInterrupt(digitalPinToInterrupt(INT_ACCEL), accel_int_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(INT_GYRO), gyro_int_isr, RISING);
    accel_timestamp = esp_timer_get_time();
    gyro_timestamp = esp_timer_get_time();
    madgwick_filter.begin(125.0f);  // Initialize Madgwick filter with 125Hz sample rate
}

void temp_task(void) {
    PID_calc(&temp_pid, get_BMI088_temperature(), TEMPERATURE_DESIRED);
    Serial.printf("Temperature: %.2f C, Heater PWM: %.2f %%\n", get_BMI088_temperature(), temp_pid.out);
    if (temp_pid.out < 0.0f) temp_pid.out = 0.0f;
    if (temp_pid.out > 100.0f) temp_pid.out = 100.0f;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, temp_pid.out);
}

void accel_callback() {
    if (!accel_data_ready) return;
    accel_data_ready = false;

    get_BMI088_accel(raw_accel);
    uint64_t now = esp_timer_get_time();
    float dt = (now - accel_timestamp) / 1000000.0f;
    imu_data.accel.dt = dt;
    accel_timestamp = now;
    accel_data_updated = true;

    float gravity[3];
    float roll = imu_data.gyro.rad_filtered[0];
    float pitch = imu_data.gyro.rad_filtered[1];
    float yaw = imu_data.gyro.rad_filtered[2];
    rotate_vector_rpy(gravity_init, roll, pitch, yaw, gravity);
    imu_data.accel.accel_no_g[0] = raw_accel[0] - gravity[0];
    imu_data.accel.accel_no_g[1] = raw_accel[1] - gravity[1];
    imu_data.accel.accel_no_g[2] = raw_accel[2] - gravity[2];

    imu_data.accel.vel[0] += imu_data.accel.accel_no_g[0] * dt;
    imu_data.accel.vel[1] += imu_data.accel.accel_no_g[1] * dt;
    imu_data.accel.vel[2] += imu_data.accel.accel_no_g[2] * dt;
    imu_data.accel.pos[0] += imu_data.accel.vel[0] * dt;
    imu_data.accel.pos[1] += imu_data.accel.vel[1] * dt;
    imu_data.accel.pos[2] += imu_data.accel.vel[2] * dt;

    std::vector<uint8_t> send_buf(
        (uint8_t*)&imu_data.accel,
        (uint8_t*)&imu_data.accel + sizeof(imu_data.accel)
    );
    std::vector<uint8_t> headers = {0x55, 0xAA, 0x55, 0xAA};
    send_buf.insert(send_buf.begin(), headers.begin(), headers.end());
    send(send_buf);
}

void gyro_callback() {
    if (!gyro_data_ready) return;
    gyro_data_ready = false;

    get_BMI088_gyro(imu_data.gyro.data);
    uint64_t now = esp_timer_get_time();
    float dt = (now - gyro_timestamp) / 1000000.0f;
    imu_data.gyro.dt = dt;
    gyro_timestamp = now;
    gyro_data_updated = true;

    integrate_gyro_quaternion(imu_data.gyro.data[0], imu_data.gyro.data[1], imu_data.gyro.data[2], dt);
    quaternion_to_euler(imu_data.gyro.rad_filtered[0], imu_data.gyro.rad_filtered[1], imu_data.gyro.rad_filtered[2]);

    std::vector<uint8_t> send_buf(
        (uint8_t*)&imu_data.gyro,
        (uint8_t*)&imu_data.gyro + sizeof(imu_data.gyro)
    );
    std::vector<uint8_t> headers = {0xAA, 0x55, 0xAA, 0x55};
    send_buf.insert(send_buf.begin(), headers.begin(), headers.end());
    send(send_buf);
}

void imu_filter_task(void) {
    if (!accel_data_updated || !gyro_data_updated) {
        Serial.println("IMU data not ready for filtering.");
        return;
    }
    accel_data_updated = false;
    gyro_data_updated = false;
    madgwick_filter.updateIMU(imu_data.gyro.data[0], imu_data.gyro.data[1], imu_data.gyro.data[2],
                              raw_accel[0], raw_accel[1], raw_accel[2]);

    imu_data.gyro.rad_filtered[0] = madgwick_filter.getRollRadians();
    imu_data.gyro.rad_filtered[1] = madgwick_filter.getPitchRadians();
    imu_data.gyro.rad_filtered[2] = madgwick_filter.getYawRadians();
}
