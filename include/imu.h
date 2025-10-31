#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    float accel_no_g[3];
    float vel[3];
    float pos[3];
    float dt;
} accel_data_t;

typedef struct {
    float data[3];
    float rad_filtered[3];
    float dt;
} gyro_data_t;

typedef struct {
    accel_data_t accel;
    gyro_data_t gyro;
} imu_t;

void heater_init(void);
void sensor_init(void);
void imu_init(void);
void gyro_callback(void);
void accel_callback(void);
void temp_task(void);
void imu_filter_task(void);

extern accel_data_t accel_data;
extern gyro_data_t gyro_data;
extern volatile bool gyro_data_ready;
extern volatile bool accel_data_ready;

#endif
