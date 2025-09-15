#include "localization.h"

extern accel_data_t accel_data;
extern gyro_data_t gyro_data;
extern wheel_delta_odom_t wheel_delta_odom;

extern HardwareSerial toRK3588Serial;

// Global state variables
static localization_state_t current_state;
static localization_filter_t filter;
static delta_pose_t last_delta;
static uint32_t last_task_timestamp;
static bool initialized = false;

void localization_init(void) {
    // Initialize state
    current_state.position = (vector3_t){0, 0, SPHERE_RADIUS}; // Start at north pole
    current_state.velocity = (vector3_t){0, 0, 0};
    current_state.orientation[0] = 1.0f; // w
    current_state.orientation[1] = 0.0f; // x
    current_state.orientation[2] = 0.0f; // y  
    current_state.orientation[3] = 0.0f; // z
    current_state.timestamp_us = 0;
    
    // Initialize Kalman filter
    memset(&filter, 0, sizeof(filter));
    
    // Set initial covariance (diagonal)
    for(int i = 0; i < 6; i++) {
        filter.covariance[i * 6 + i] = 1.0f;
    }
    
    // Initialize bias estimation with calibration
    filter.calibration_samples = 0;
    filter.accel_sum = (vector3_t){0, 0, 0};
    filter.gyro_sum = (vector3_t){0, 0, 0};
    
    last_delta = (delta_pose_t){0, 0, 0, 0};
    last_task_timestamp = 0;
    initialized = true;
}

void localization_task(void) {
    if (!initialized) {
        localization_init();
        return;
    }
    
    uint32_t current_time = micros();
    float dt = (current_time - last_task_timestamp) / 1000000.0f; // Convert to seconds
    
    if (last_task_timestamp == 0) {
        last_task_timestamp = current_time;
        return;
    }
    
    // Bias calibration phase (first 2 seconds)
    if (filter.calibration_samples < 2000) {
        calibrate_sensors();
        last_task_timestamp = current_time;
        return;
    }
    
    // Get sensor data
    vector3_t accel = {accel_data.x, accel_data.y, accel_data.z};
    vector3_t gyro = {gyro_data.x, gyro_data.y, gyro_data.z};
    
    // Remove bias
    accel.x -= filter.accel_bias.x;
    accel.y -= filter.accel_bias.y;
    accel.z -= filter.accel_bias.z;
    
    gyro.x -= filter.gyro_bias.x;
    gyro.y -= filter.gyro_bias.y;
    gyro.z -= filter.gyro_bias.z;
    
    // Prediction step
    kalman_predict(dt);
    
    // Update with IMU data
    kalman_update_imu(accel, gyro, dt);
    
    // Update with odometry
    float odom_dt = 1.0f / SYSTICK;
    kalman_update_odometry(wheel_delta_odom.x, wheel_delta_odom.rot, odom_dt);
    
    // Calculate delta pose from previous state
    spherical_pose_t current_spherical;
    cartesian_to_spherical(current_state.position, &current_spherical.theta, &current_spherical.phi);
    current_spherical.heading = filter.state[4];
    
    // Store delta for external access
    last_delta.delta_theta = filter.state[0] - (last_delta.delta_theta + filter.state[0]);
    last_delta.delta_phi = filter.state[1] - (last_delta.delta_phi + filter.state[1]);
    last_delta.delta_heading = filter.state[4] - (last_delta.delta_heading + filter.state[4]);
    
    // Calculate confidence based on covariance trace
    float trace = filter.covariance[0] + filter.covariance[7] + filter.covariance[14] + 
                  filter.covariance[21] + filter.covariance[28] + filter.covariance[35];
    last_delta.confidence = 1.0f / (1.0f + trace);
    
    // Update timestamp
    last_task_timestamp = current_time;
    current_state.timestamp_us = current_time;

    // Send updated state
    toRK3588Serial.write(0x10);
    toRK3588Serial.write((uint8_t*)&last_delta, sizeof(last_delta));
}

static void calibrate_sensors(void) {
    filter.accel_sum.x += accel_data.x;
    filter.accel_sum.y += accel_data.y;
    filter.accel_sum.z += accel_data.z;
    
    filter.gyro_sum.x += gyro_data.x;
    filter.gyro_sum.y += gyro_data.y;
    filter.gyro_sum.z += gyro_data.z;
    
    filter.calibration_samples++;
    
    if (filter.calibration_samples >= 2000) {
        // Calculate bias (assuming robot is stationary during calibration)
        filter.accel_bias.x = filter.accel_sum.x / filter.calibration_samples;
        filter.accel_bias.y = filter.accel_sum.y / filter.calibration_samples;
        filter.accel_bias.z = (filter.accel_sum.z / filter.calibration_samples) - 9.81f; // Remove gravity
        
        filter.gyro_bias.x = filter.gyro_sum.x / filter.calibration_samples;
        filter.gyro_bias.y = filter.gyro_sum.y / filter.calibration_samples;
        filter.gyro_bias.z = filter.gyro_sum.z / filter.calibration_samples;
    }
}

static void kalman_predict(float dt) {
    // State prediction: [theta, phi, vel_theta, vel_phi, heading, heading_rate]
    float dt2 = dt * dt;
    
    // Position prediction with spherical constraints
    filter.state[0] += filter.state[2] * dt; // theta += vel_theta * dt
    filter.state[1] += filter.state[3] * dt / sin(filter.state[0]); // phi += vel_phi * dt / sin(theta)
    filter.state[4] += filter.state[5] * dt; // heading += heading_rate * dt
    
    // Wrap angles
    while (filter.state[1] > M_PI) filter.state[1] -= 2 * M_PI;
    while (filter.state[1] < -M_PI) filter.state[1] += 2 * M_PI;
    while (filter.state[4] > M_PI) filter.state[4] -= 2 * M_PI;
    while (filter.state[4] < -M_PI) filter.state[4] += 2 * M_PI;
    
    // Clamp theta to valid range [0, π]
    if (filter.state[0] < 0) filter.state[0] = 0;
    if (filter.state[0] > M_PI) filter.state[0] = M_PI;
    
    // Covariance prediction (simplified)
    for (int i = 0; i < 6; i++) {
        filter.covariance[i * 6 + i] += KALMAN_Q_PROCESS * dt2;
    }
}

static void kalman_update_imu(vector3_t accel, vector3_t gyro, float dt) {
    // Transform accelerometer reading to spherical coordinates
    float theta = filter.state[0];
    float phi = filter.state[1];
    
    // Rotate acceleration to tangent plane
    rotate_vector_to_tangent_plane(&accel, theta, phi);
    
    // Update velocity estimates (simplified Kalman update)
    float innovation_theta = accel.x * dt - (filter.state[2] - filter.state[2]); // Acceleration integration
    float innovation_phi = accel.y * dt - (filter.state[3] - filter.state[3]);
    float innovation_heading = gyro.z - filter.state[5];
    
    // Kalman gain (simplified)
    float K = KALMAN_Q_PROCESS / (KALMAN_Q_PROCESS + KALMAN_R_MEASUREMENT);
    
    // State update
    filter.state[2] += K * innovation_theta; // vel_theta
    filter.state[3] += K * innovation_phi;   // vel_phi  
    filter.state[5] += K * innovation_heading; // heading_rate
    
    // Covariance update
    for (int i = 0; i < 6; i++) {
        filter.covariance[i * 6 + i] *= (1.0f - K);
    }
}

static void kalman_update_odometry(float delta_x, float delta_rot, float dt) {
    if (dt <= 0) return;
    
    // Convert wheel odometry to spherical velocity
    float vel_linear = delta_x / dt;
    float vel_angular = delta_rot / dt;
    
    // Map to spherical coordinates
    float vel_theta = vel_linear * cos(filter.state[4]) / SPHERE_RADIUS;
    float vel_phi = vel_linear * sin(filter.state[4]) / (SPHERE_RADIUS * sin(filter.state[0]));
    
    // Measurement innovation
    float innovation_vel_theta = vel_theta - filter.state[2];
    float innovation_vel_phi = vel_phi - filter.state[3];
    float innovation_heading_rate = vel_angular - filter.state[5];
    
    // Kalman gain for odometry (higher trust)
    float K_odom = 0.7f;
    
    // Update state
    filter.state[2] += K_odom * innovation_vel_theta;
    filter.state[3] += K_odom * innovation_vel_phi;
    filter.state[5] += K_odom * innovation_heading_rate;
    
    // Update covariance (reduce uncertainty)
    filter.covariance[14] *= (1.0f - K_odom); // vel_theta variance
    filter.covariance[21] *= (1.0f - K_odom); // vel_phi variance
    filter.covariance[35] *= (1.0f - K_odom); // heading_rate variance
}

static void rotate_vector_to_tangent_plane(vector3_t* vec, float theta, float phi) {
    // Rotation matrix to convert from global frame to local tangent plane
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float cos_phi = cos(phi);
    float sin_phi = sin(phi);
    
    float x = vec->x;
    float y = vec->y;
    float z = vec->z;
    
    // Transform to tangent plane coordinates
    vec->x = cos_phi * cos_theta * x + sin_phi * cos_theta * y - sin_theta * z;
    vec->y = -sin_phi * x + cos_phi * y;
    vec->z = cos_phi * sin_theta * x + sin_phi * sin_theta * y + cos_theta * z;
}

static vector3_t spherical_to_cartesian(float theta, float phi) {
    return (vector3_t){
        SPHERE_RADIUS * sin(theta) * cos(phi),
        SPHERE_RADIUS * sin(theta) * sin(phi), 
        SPHERE_RADIUS * cos(theta)
    };
}

static void cartesian_to_spherical(vector3_t cart, float* theta, float* phi) {
    float r = sqrt(cart.x * cart.x + cart.y * cart.y + cart.z * cart.z);
    *theta = acos(cart.z / r);
    *phi = atan2(cart.y, cart.x);
}

// Public API functions
delta_pose_t localization_get_delta_pose(void) {
    return last_delta;
}

spherical_pose_t localization_get_current_pose(void) {
    spherical_pose_t pose;
    cartesian_to_spherical(current_state.position, &pose.theta, &pose.phi);
    pose.heading = filter.state[4];
    return pose;
}

void localization_reset_pose(spherical_pose_t initial_pose) {
    current_state.position = spherical_to_cartesian(initial_pose.theta, initial_pose.phi);
    filter.state[0] = initial_pose.theta;
    filter.state[1] = initial_pose.phi;
    filter.state[4] = initial_pose.heading;
    // Reset velocities
    filter.state[2] = filter.state[3] = filter.state[5] = 0;
}