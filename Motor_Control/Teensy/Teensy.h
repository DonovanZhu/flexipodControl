#ifndef __Teensy_H
#define __Teensy_H

// Defines
#define MOTOR_NUM          12                 // Number of ESCs
#define USB_UART_SPEED     1000000            // Baudrate of the teeensy USB serial link

#define REDUCTION_RATIO        8.0  // Drive ratio of motor gear box

// -0.0192715 -0.2800474 10.0537329 0.0136186 0.0011329 -0.0076148
#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET      0.0131094
#define GYRO_Y_OFFSET      0.0012340
#define GYRO_Z_OFFSET     -0.0056364

#define ACCEL_X_OFFSET     0.0426984
#define ACCEL_Y_OFFSET    -0.3458546
#define ACCEL_Z_OFFSET     10.0628948

// Teensy->host communication data structure
typedef struct {
  float    joint_pos[MOTOR_NUM];      // Motors rotation angle
  float    joint_vel[MOTOR_NUM];     // Motors rad/s
  float    joint_cur[MOTOR_NUM];     // Motors current
  float    acc[3];             // Acceleration in X Y Z direction, m/s^2
  float    gyr[3];             // Gyroscope in X Y Z direction, deg/s
  float    mag[3];             // Magnetometer in X Y Z, uT
  float    eular[3];
  float    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  float    comd[MOTOR_NUM];            // Desired position, rad
} Jetson_comm_struct_t;

struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};

void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);

EulerAngles ToEulerAngles(Quaternion q);
#endif
