#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define SYSTICK 1000  // 1ms

typedef struct {
  uint8_t header; // 0x03
  double vel_x;
  double vel_rot;
  uint16_t checksum;
} __attribute__((packed)) control_data_t;

typedef struct {
  uint8_t header; // 0xAA
  uint8_t x;
  uint8_t y;
  uint8_t tail;
} rc_data_t;

typedef struct {
  uint8_t header; // 0x04
  double x;
  double rot;
  uint16_t checksum;
} wheel_delta_odom_t;

#endif
