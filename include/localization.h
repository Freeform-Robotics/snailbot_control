#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "main.h"
#include "imu.h"

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Spherical surface parameters
#define SPHERE_RADIUS 0.06f  // meters - adjust based on your sphere

// Filter parameters
#define KALMAN_Q_PROCESS 0.01f    // Process noise
#define KALMAN_R_MEASUREMENT 0.1f  // Measurement noise
#define ALPHA_COMP_FILTER 0.98f    // Complementary filter weight

typedef struct {
    float x, y, z;
} vector3_t;

typedef struct {
    float theta;    // Latitude (angle from north pole)
    float phi;      // Longitude
    float heading;  // Heading angle on tangent plane
} spherical_pose_t;

typedef struct {
    float delta_theta;  // Change in latitude
    float delta_phi;    // Change in longitude  
    float delta_heading; // Change in heading
    float confidence;   // Confidence measure [0,1]
} delta_pose_t;

typedef struct {
    vector3_t position;     // Current estimated position on sphere
    vector3_t velocity;     // Current velocity
    float orientation[4];   // Quaternion orientation
    uint32_t timestamp_us;  // Last update timestamp
} localization_state_t;

typedef struct {
    // Kalman filter state
    float state[6];         // [pos_theta, pos_phi, vel_theta, vel_phi, heading, heading_rate]
    float covariance[36];   // 6x6 covariance matrix
    
    // Integration variables
    vector3_t accel_integrated;
    vector3_t gyro_integrated;
    
    // Bias estimation
    vector3_t accel_bias;
    vector3_t gyro_bias;
    
    // Calibration state
    uint16_t calibration_samples;
    vector3_t accel_sum;
    vector3_t gyro_sum;
    
} localization_filter_t;

// Function declarations
void localization_init(void);
void localization_task(void);
delta_pose_t localization_get_delta_pose(void);
spherical_pose_t localization_get_current_pose(void);
void localization_reset_pose(spherical_pose_t initial_pose);

// Internal functions
static void kalman_predict(float dt);
static void kalman_update_imu(vector3_t accel, vector3_t gyro, float dt);
static void kalman_update_odometry(float delta_x, float delta_rot, float dt);
static vector3_t spherical_to_cartesian(float theta, float phi);
static void cartesian_to_spherical(vector3_t cart, float* theta, float* phi);
static void rotate_vector_to_tangent_plane(vector3_t* vec, float theta, float phi);
static void calibrate_sensors(void);

#endif