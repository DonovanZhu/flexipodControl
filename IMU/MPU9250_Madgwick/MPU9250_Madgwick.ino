#include "MPU9250.h"
#include "MadgwickAHRS.h"
MPU9250 IMU(Wire,0x68);

#define GRAVITY 9.802 // The gravity acceleration in New York City

//0.1466798 0.2581153 -10.2391586 -0.0001533 -0.0003119 -0.0000189

    


// Calibration outcomes
//#define GYRO_X_OFFSET -0.0002077
//#define GYRO_Y_OFFSET -0.0002122
//#define GYRO_Z_OFFSET 0.0010441

#define ACCEL_X_OFFSET 0.1909475
#define ACCEL_Y_OFFSET 0.10116055
#define ACCEL_Z_OFFSET -0.2328835

float GYRO_X_OFFSET = 0.0;
float GYRO_Y_OFFSET = 0.0;
float GYRO_Z_OFFSET = 0.0;
//
//float ACCEL_X_OFFSET = 0.0;
//float ACCEL_Y_OFFSET = 0.0;
//float ACCEL_Z_OFFSET = 0.0;

float K[3][3] = {{0.9991416, 0.0, 0.0}, {0.0, 0.9949671162, 0.0}, {0.0, 0.0, 0.98455696}};
float b[3] = {0.1909475, 0.10116055, -0.2328835};

const float magn_ellipsoid_center[3] = {4.23218, -7.60568, -18.7859};
const float magn_ellipsoid_transform[3][3] = {{0.880559, -0.00714649, 0.00976959}, {-0.00714649, 0.995225, -0.0131647}, {0.00976959, -0.0131647, 0.955716}};
// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_b[3];
float magnetom[3];
float magnetom_tmp[3];
float gyro[3];

float time_now;
float time_former;
float deltat;

// Set structs for converting result from Quaternion to Euler angles
struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};

Quaternion qua;
EulerAngles eul;

// Read data from MPU9250
void read_sensors() {
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();

  magnetom[0] = IMU.getMagX_uT();
  magnetom[1] = IMU.getMagY_uT();
  magnetom[2] = IMU.getMagZ_uT();

  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}


void sensor_init() {
  for (int i = 0; i < 30000; ++i) {
    IMU.readSensor();
    GYRO_X_OFFSET += IMU.getGyroX_rads();
    GYRO_Y_OFFSET += IMU.getGyroY_rads();
    GYRO_Z_OFFSET += IMU.getGyroZ_rads();
  }
  GYRO_X_OFFSET /= 30000.0;
  GYRO_Y_OFFSET /= 30000.0;
  GYRO_Z_OFFSET /= 30000.0;
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
//    accel[0] = accel[0] - ACCEL_X_OFFSET;
//    accel[1] = accel[1] - ACCEL_Y_OFFSET;
//    accel[2] = accel[2] - ACCEL_Z_OFFSET;
    accel_b[0] = accel[0] - b[0];
    accel_b[1] = accel[1] - b[1];
    accel_b[2] = accel[2] - b[2];
    Matrix_Vector_Multiply(K, accel_b, accel);

    
    
    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);

    // Compensate gyroscope error
    gyro[0] -= GYRO_X_OFFSET;
    gyro[1] -= GYRO_Y_OFFSET;
    gyro[2] -= GYRO_Z_OFFSET;
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

void sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}



EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll_e = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch_e = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch_e = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw_e = atan2(siny_cosp, cosy_cosp);

    return angles;
}

void setup() {
  pinMode(13, OUTPUT);  
  digitalWrite(13, HIGH);
  Serial.begin(1000000);
  while (!Serial) yield();

  IMU.begin();

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 41 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(9);
  

  sensor_init();
  digitalWrite(13, LOW);
  time_former = micros();
}


// -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz
void loop() {

  // put your main code here, to run repeatedly:
  read_sensors();

  compensate_sensor_errors();

  time_now = micros();
  deltat = (float)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
  MadgwickQuaternionUpdate(accel[0], accel[1], accel[2],
                           gyro[0], gyro[1], gyro[2],
                           magnetom[0], magnetom[1], magnetom[2], deltat);
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.8;
  if (eul.yaw_e > PI) {
    eul.yaw_e -= 2 * PI;
  }
//  Serial.print(eul.roll_e * 180.0 / PI);
//  Serial.print(" ");
//  Serial.print(eul.pitch_e * 180.0 / PI);
//  Serial.print(" ");
//  Serial.println(eul.yaw_e * 180.0 / PI);
//  Serial.println(" ");
//  Serial.print(ACCEL_X_OFFSET,7);
//  Serial.print(" ");
//  Serial.print(ACCEL_Y_OFFSET,7);
//  Serial.print(" ");
//  Serial.print(ACCEL_Z_OFFSET,7);
//  Serial.println(" ");
  sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  //delayMicroseconds(500);
}
