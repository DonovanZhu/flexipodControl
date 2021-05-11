#ifndef __Teensy_H
#define __Teensy_H

// Defines
#define MOTOR_NUM                 12                 // Number of ESCs
#define USB_UART_SPEED            1000000            // Baudrate of the teeensy USB serial link
#define REDUCTION_RATIO           8.0  // Drive ratio of motor gear box
#define GRAVITY                   9.802 // The gravity acceleration in New York City
#define GYRO_CALIBRATION_LOOP_NUM 30000.0

// Teensy->host communication data structure
typedef struct {
  float    joint_pos[MOTOR_NUM];      // Motors rotation angle
  float    joint_vel[MOTOR_NUM];     // Motors rad/s
  float    joint_cur[MOTOR_NUM];     // Motors current
  float    acc[3];             // Acceleration in X Y Z direction, m/s^2
  float    gyr[3];             // Gyroscope in X Y Z direction, deg/s
  float    mag[3];             // Magnetometer in X Y Z, uT
  float    euler[3];
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
