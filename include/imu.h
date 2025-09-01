#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    uint8_t header;     // 0x01
    float accel_x;
    float accel_y;
    float accel_z;
    uint16_t checksum;
} accel_data_t;

typedef struct {
    uint8_t header;     // 0x02
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint16_t checksum;
} gyro_data_t;

void heater_init(void);
void sensor_init(void);
void imu_init(void);
void gyro_callback(void);
void accel_callback(void);
void temp_task(void);

#endif