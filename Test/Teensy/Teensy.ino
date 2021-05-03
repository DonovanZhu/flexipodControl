#include "Teensy.h"


// Globals
float joint_pos_desired[MOTOR_NUM]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // desired joint(motor) position [rad]
float rotor_pos[MOTOR_NUM];
float rotor_pos_prev[MOTOR_NUM];
int   r_num[MOTOR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float time_now;
float time_former;

// Motor shaft position [rad]
float joint_pos[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Motor shaft velocity [rad/s]
float joint_vel[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Motor current [A]
float joint_cur[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float joint_upper_limit[MOTOR_NUM] = { 2.4,  1.93,  1.6,  4.7,  1.93,  1.6,  4.7,  1.93,  1.6,  2.4,  1.93,  1.6};
float joint_lower_limit[MOTOR_NUM] = {-4.7, -1.93, -1.6, -2.4, -1.93, -1.6, -2.4, -1.93, -1.6, -4.7, -1.93, -1.6};

float delta_t; // loop time difference
Teensycomm_struct_t   teensy_comm = {{}, {}, {}, {}, {}, {}, {}, {}};   // For holding data sent to Jetson
Jetson_comm_struct_t  jetson_comm    = {{}};                   // For holding data received from Jetson

static uint8_t  *ptin  = (uint8_t*)(&jetson_comm);
static uint8_t  *ptout = (uint8_t*)(&teensy_comm);
static int      in_cnt = 0;


// lookup table to relate motor index i to the CAN bus and its motor CAN address
// 0 stands for CAN_F, 1 stands for CAN_B
bool joint_can_lane [MOTOR_NUM] = { 0,0,0,0,0,0,
                                    1,1,1,1,1,1};
// Motors' id from 0x141 to 0x146
uint16_t joint_can_addr[MOTOR_NUM] = {0x141, 0x142, 0x143, 0144, 0x145, 0x146, 
                                      0x141, 0x142, 0x143, 0144, 0x145, 0x146};


void Jetson_Teensy () {
  in_cnt = 0;


  // Read all incoming bytes available until incoming structure is complete
  while ((Serial.available() > 0) && (in_cnt < (int)sizeof(jetson_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(jetson_comm)) {

    // Clear incoming bytes counter
    in_cnt = 0;

    // Save angle, speed and torque into struct teensy_comm
    for (int i = 0; i < MOTOR_NUM; i++) {
      teensy_comm.joint_pos[i]  = 0.0;
      teensy_comm.joint_vel[i] = 1.0;
      teensy_comm.joint_cur[i] = 2.0;
    }

    // Read data from IMU(MPU9250)
   // Save acceleration (m/s^2) of IMU into struct teensy_comm
    teensy_comm.acc[0] = 3.0;
    teensy_comm.acc[1] = 4.0;
    teensy_comm.acc[2] = 5.0;

    // Save gyroscope (rad/s) of IMU into struct teensy_comm
    teensy_comm.gyr[0] = 6.0;
    teensy_comm.gyr[1] = 7.0;
    teensy_comm.gyr[2] = 8.0;

    teensy_comm.mag[0] = 9.0;
    teensy_comm.mag[1] = 10.0;
    teensy_comm.mag[2] = 11.0;

    teensy_comm.eular[0] = jetson_comm.comd[0];
    teensy_comm.eular[1] = jetson_comm.comd[1];
    teensy_comm.eular[2] = jetson_comm.comd[2];

    teensy_comm.timestamps = time_now / 1000000.0;
    // Send data structure teensy_comm to Jetson
    Serial.write(ptout, sizeof(teensy_comm));

    // Force immediate transmission
    Serial.send_now();
  }
}



void setup() {
  // Switch on CAN bus
  Serial.begin(USB_UART_SPEED);
  delay(1000);

  time_former = (float)micros();
}

void loop() {
  
  Jetson_Teensy ();
  delayMicroseconds(2000);
}
