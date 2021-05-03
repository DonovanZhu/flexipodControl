#ifndef __Teensy_H
#define __Teensy_H

// Defines
#define MOTOR_NUM          12                 // Number of ESCs
#define USB_UART_SPEED     1000000            // Baudrate of the teeensy USB serial link

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
#endif
