#include <FlexCAN_T4.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include "MadgwickAHRS.h"
#include "Teensy.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_F; // CAN bus for upper body motors
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_B; // CAN bus for lower body motors


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

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus

// lookup table to relate motor index i to the CAN bus and its motor CAN address
// 0 stands for CAN_F, 1 stands for CAN_B
bool joint_can_lane [MOTOR_NUM] = { 0,0,0,0,0,0,
                                    1,1,1,1,1,1};
// Motors' id from 0x141 to 0x146
uint16_t joint_can_addr[MOTOR_NUM] = {0x141, 0x142, 0x143, 0x144, 0x145, 0x146, 
                                      0x141, 0x142, 0x143, 0x144, 0x145, 0x146};
/****************** IMU ***************************************/
Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t acc;
sensors_event_t gyr;
sensors_event_t temp;
sensors_event_t mag;

float gyro_x_offset = 0.0;
float gyro_y_offset = 0.0;
float gyro_z_offset = 0.0;

float acc_transform[3][3] = {{0.9991416, 0.0, 0.0}, {0.0, 0.9949671162, 0.0}, {0.0, 0.0, 0.98455696}};
float acc_offset[3] = {0.1909475, 0.10116055, -0.2328835};

const float magn_ellipsoid_center[3] = {4.23218, -7.60568, -18.7859};
const float magn_ellipsoid_transform[3][3] = {{0.880559, -0.00714649, 0.00976959}, {-0.00714649, 0.995225, -0.0131647}, {0.00976959, -0.0131647, 0.955716}};

float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_b[3];
float magnetom[3];
float magnetom_tmp[3];
float gyro[3];

Quaternion qua;
EulerAngles eul;

void read_sensors() {
  sox.getEvent(&acc, &gyr, &temp);
  lis.getEvent(&mag);
  accel[0] = acc.acceleration.x;
  accel[1] = acc.acceleration.y;
  accel[2] = acc.acceleration.z;

  magnetom[0] = mag.magnetic.x;
  magnetom[1] = mag.magnetic.y;
  magnetom[2] = mag.magnetic.z;

  gyro[0] = gyr.gyro.x;
  gyro[1] = gyr.gyro.y;
  gyro[2] = gyr.gyro.z;
}

void sensor_init() {
  for (int i = 0; i < GYRO_CALIBRATION_LOOP_NUM; ++i) {
    sox.getEvent(&acc, &gyr, &temp);
    gyro_x_offset += gyr.gyro.x;
    gyro_y_offset += gyr.gyro.y;
    gyro_z_offset += gyr.gyro.z;
  }
  gyro_x_offset /= GYRO_CALIBRATION_LOOP_NUM;
  gyro_y_offset /= GYRO_CALIBRATION_LOOP_NUM;
  gyro_z_offset /= GYRO_CALIBRATION_LOOP_NUM;
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel_b[0] = accel[0] - acc_offset[0];
    accel_b[1] = accel[1] - acc_offset[1];
    accel_b[2] = accel[2] - acc_offset[2];
    Matrix_Vector_Multiply(acc_transform, accel_b, accel);   
    
    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);

    // Compensate gyroscope error
    gyro[0] -= gyro_x_offset;
    gyro[1] -= gyro_y_offset;
    gyro[2] -= gyro_z_offset;
}
/*********************************************************/

void Motor_Init() {
  // Motor position initial CAN bus command
  // All motors rotate to position 0 rad
  msg_send.buf[0] = 0xA3; //CAN bus position command ID
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = 0x00;
  msg_send.buf[5] = 0x00;
  msg_send.buf[6] = 0x00;
  msg_send.buf[7] = 0x00;

  // Motors' IDs range from 0x141 to 0x146.
  // CAN_F is connected to upper body motors(id: 0x141 to 0x146)
  // CAN_B is connected to lower body motors(id: 0x141 to 0x146)
  for (int i = 0; i < MOTOR_NUM / 2; ++i) {
      msg_send.id = joint_can_addr[i];
      CAN_F.write(msg_send);
      msg_send.id = joint_can_addr[i + MOTOR_NUM / 2];
      CAN_B.write(msg_send);
  }
  delay(400);
}

void processMotorData(int id) {
  // Receiving motor angle
  // Transfer hex number to rad
  int rotor_pos_raw = 0; // Rotor position before devided by gear reduction
  rotor_pos_raw |= (int16_t)(unsigned char)msg_recv.buf[7] << 8; // Left move 8 bit
  rotor_pos_raw |= (int16_t)(unsigned char)msg_recv.buf[6];
  rotor_pos[id] = (float)rotor_pos_raw / 65535.0 * 2 * PI; // 65535(0xFFFF) refers to 2PI
  if (rotor_pos[id] - rotor_pos_prev[id] < -PI)
    r_num[id] += 1;
  else if (rotor_pos[id] - rotor_pos_prev[id] > PI)
    r_num[id] -= 1;

  // Calculate shaft angular position [rad]
  rotor_pos_prev[id] = rotor_pos[id];
  joint_pos[id] = (rotor_pos [id]+ r_num[id] * 2 * PI) / REDUCTION_RATIO;

  // Calculate shaft velocity [rad/s]
  int rotor_vel_raw = 0;
  rotor_vel_raw |= (int16_t)(unsigned char)msg_recv.buf[5] << 8;
  rotor_vel_raw |= (int16_t)(unsigned char)msg_recv.buf[4];

  // 0x0001 to 0x8000 counter-clockwise, 0x8000: max counter-clockwise speed
  // 0x8001 to 0xffff clockwise, 0x8001: max clockwise speed
  // 0x0000 refers to stop
  if (rotor_vel_raw > 0x8000)
    rotor_vel_raw -= 0x10000;
  joint_vel[id] = (float)rotor_vel_raw * PI / (180.0 * REDUCTION_RATIO);

  // Calculate motor's current [A], -33A ~ 33A
  int cur_raw = 0;
  cur_raw |= (int16_t)(unsigned char)msg_recv.buf[3] << 8;
  cur_raw |= (int16_t)(unsigned char)msg_recv.buf[2];
  if (cur_raw > 0x8000)
    cur_raw -= 0x10000;
  joint_cur[id] = (float)cur_raw * 33.0 / 2048.0; // 2048 refers to 33A
}

void Angle_Control_Loop(int motor_id, float pos_command) {
  // Convert motor shaft angle command [rad] to rotor angle command [degree]
  pos_command = pos_command * 180.0 * REDUCTION_RATIO / PI;
  // see motor manual p12 (0xA3)
  int32_t pos = (int32_t)round(pos_command / 0.01);
  
  // Motor position command is clockwise
  // 0x00000001 - 0x80000000 counter_clockwise
  // 0x80000001 - 0xffffffff clockwise
  // 0x00000000 position 0
  if (pos < 0)
    pos = 0x100000000 + pos;

  unsigned int pos_1 = pos & 0xff;
  unsigned int pos_2 = (pos >> 8) & 0xff;
  unsigned int pos_3 = (pos >> 16) & 0xff;
  unsigned int pos_4 = (pos >> 24) & 0xff;

  // Set the CAN message ID as 0x200
  msg_send.id = joint_can_addr[motor_id];
  msg_send.buf[0] = 0xA3;
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = pos_1;
  msg_send.buf[5] = pos_2;
  msg_send.buf[6] = pos_3;
  msg_send.buf[7] = pos_4;


  if (joint_can_lane[motor_id]==0) {
    CAN_F.write(msg_send); // Write the message on CAN bus
    while (true) {
      if (CAN_F.read(msg_recv)) {
        processMotorData(motor_id);
        break;
      }
    }
  }
    
  else {
    CAN_B.write(msg_send); // Write the message on CAN bus
    while (true) {
      if (CAN_B.read(msg_recv)) {
        processMotorData(motor_id);
        break;
      }
    }
  }
}

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
      teensy_comm.joint_pos[i]  = joint_pos[i];
      teensy_comm.joint_vel[i] = joint_vel[i];
      teensy_comm.joint_cur[i] = joint_cur[i];
    }

    // Read data from IMU(MPU9250)
   // Save acceleration (m/s^2) of IMU into struct teensy_comm
    teensy_comm.acc[0] = accel[0];
    teensy_comm.acc[1] = accel[1];
    teensy_comm.acc[2] = accel[2];

    // Save gyroscope (rad/s) of IMU into struct teensy_comm
    teensy_comm.gyr[0] = gyro[0] / 180.0 * PI;
    teensy_comm.gyr[1] = gyro[1] / 180.0 * PI;
    teensy_comm.gyr[2] = gyro[2] / 180.0 * PI;

    teensy_comm.mag[0] = magnetom[0];
    teensy_comm.mag[1] = magnetom[1];
    teensy_comm.mag[2] = magnetom[2];

    teensy_comm.euler[0] = eul.roll_e;
    teensy_comm.euler[1] = eul.pitch_e;
    teensy_comm.euler[2] = eul.yaw_e;

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

  pinMode(13, OUTPUT);  
  digitalWrite(13, HIGH);


  sox.begin_I2C();
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_52_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_52_HZ);

  lis.begin_I2C();          // hardware I2C mode, can pass in address & alt Wire
  lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis.setDataRate(LIS3MDL_DATARATE_40_HZ);
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis.setIntThreshold(500);
  lis.configInterrupt(false, false, true, true, false, true); // enabled!
  
  sensor_init();
  digitalWrite(13, LOW);

  CAN_F.begin();
  CAN_F.setBaudRate(1000000);
  CAN_F.setClock(CLK_60MHz);

  CAN_B.begin();
  CAN_B.setBaudRate(1000000);
  CAN_B.setClock(CLK_60MHz);
  // Open Serial port in speed 1000000 Baudrate
  Motor_Init();
  
  time_former = (float)micros();
}

void loop() {

  read_sensors();
  
  Jetson_Teensy ();
  
  compensate_sensor_errors();
  
  time_now = (float)micros();
  delta_t = (time_now - time_former) / 1000000.0;
  time_former = time_now;

  MadgwickQuaternionUpdate(accel[0], accel[1], accel[2],
                           gyro[0], gyro[1], gyro[2],
                           magnetom[0], magnetom[1], magnetom[2], delta_t);
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.22; // 0.22 rad is the Magnetic Declination in New York
  if (eul.yaw_e > PI) {
    eul.yaw_e -= 2 * PI;
  }

  
  for (int i = 0; i < MOTOR_NUM; ++i) {
    joint_pos_desired[i] = jetson_comm.comd[i];
    if (joint_pos_desired[i] > joint_upper_limit[i])
      joint_pos_desired[i] = joint_upper_limit[i];
    else if (joint_pos_desired[i] < joint_lower_limit[i])
      joint_pos_desired[i] = joint_lower_limit[i];
  }
  
  for (int i = 0; i < MOTOR_NUM / 2; ++i){
      Angle_Control_Loop(i, joint_pos_desired[i]);
      int j = i + MOTOR_NUM / 2;
      Angle_Control_Loop(j, joint_pos_desired[j]);
  }
}
