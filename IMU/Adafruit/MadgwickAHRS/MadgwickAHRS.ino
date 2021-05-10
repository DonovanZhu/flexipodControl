#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include "MadgwickAHRS.h"

Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mage;
sensors_event_t temp;

// [ 1.0057926403, -0.0163004481, -0.0069086382, 
//   0.0103163034, 0.9883109468, 0.0101499793, 
//   0.0032268310, 0.0037796109, 0.9930840134]
#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
// 0.0118531 0.0028559 -0.0067276
#define GYRO_X_OFFSET 0.0118531
#define GYRO_Y_OFFSET 0.0028559
#define GYRO_Z_OFFSET -0.0067276

//-10.0869466 9.732806
//9.9026966 -9.7519484
// -9.7686987 10.1431761
float K[3][3] = {{ 1.0057926403, -0.0163004481, -0.0069086382}, 
{0.0103163034, 0.9883109468, 0.0101499793}, 
{0.0032268310, 0.0037796109, 0.9930840134}};
float bias[3] = {-0.0245, 0.1103, 0.1373};

#define AVG_NUM 10

const float magn_ellipsoid_center[3] = {-23.8724, 19.4826, -9.07726};
const float magn_ellipsoid_transform[3][3] = {{0.884691, 0.0385287, -0.00557409}, {0.0385287, 0.904418, 0.0782215}, {-0.00557409, 0.0782215, 0.929234}};
// Sensor variables
float acc[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_b[3];
float mag[3];
float mag_tmp[3];
float gyr[3];

float time_now;
float time_former;
float deltat;

float rotation_matrix[3][3] = {0};

float acc_avg[AVG_NUM][3];
float gyr_avg[AVG_NUM][3];
float mag_avg[AVG_NUM][3];

// Set structs for converting result from Quaternion to Euler angles
struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};

Quaternion qua;
EulerAngles eul;

void read_sensors() {
  sox.getEvent(&accel, &gyro, &temp);
  lis.getEvent(&mage);
  for (int i = 0; i < AVG_NUM - 1; ++i) {
    for (int j = 0; j < 3; ++j) {
      acc_avg[i][j] = acc_avg[i + 1][j];
      gyr_avg[i][j] = gyr_avg[i + 1][j];
      mag_avg[i][j] = mag_avg[i + 1][j];
    }
  }
  acc_avg[AVG_NUM - 1][0] = accel.acceleration.x;
  acc_avg[AVG_NUM - 1][1] = accel.acceleration.y;
  acc_avg[AVG_NUM - 1][2] = accel.acceleration.z;

  mag_avg[AVG_NUM - 1][0] = mage.magnetic.x;
  mag_avg[AVG_NUM - 1][1] = mage.magnetic.y;
  mag_avg[AVG_NUM - 1][2] = mage.magnetic.z;

  gyr_avg[AVG_NUM - 1][0] = gyro.gyro.x;
  gyr_avg[AVG_NUM - 1][1] = gyro.gyro.y;
  gyr_avg[AVG_NUM - 1][2] = gyro.gyro.z;

  for (int i = 0; i < AVG_NUM; ++i) {
    for (int j = 0; j < 3; ++j) {
      acc[j] += acc_avg[i][j];
      gyr[j] += gyr_avg[i][j];
      mag[j] += mag_avg[i][j];
    }
  }
  for (int j = 0; j < 3; ++j) {
    acc[j] /= AVG_NUM;
    gyr[j] /= AVG_NUM;
    mag[j] /= AVG_NUM;
  }
}

void data_init() {
  for (int i = 0; i < AVG_NUM; ++i) {
    sox.getEvent(&accel, &gyro, &temp);
    lis.getEvent(&mage);
    acc_avg[i][0] = accel.acceleration.x;
    acc_avg[i][1] = accel.acceleration.y;
    acc_avg[i][2] = accel.acceleration.z;

    gyr_avg[i][0] = gyro.gyro.x;
    gyr_avg[i][1] = gyro.gyro.y;
    gyr_avg[i][2] = gyro.gyro.z;

    mag_avg[i][0] = mage.magnetic.x;
    mag_avg[i][1] = mage.magnetic.y;
    mag_avg[i][2] = mage.magnetic.z;
  }
}

void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel_b[0] = acc[0] - bias[0];
    accel_b[1] = acc[1] - bias[1];
    accel_b[2] = acc[2] - bias[2];
    Matrix_Vector_Multiply(K, accel_b, acc);   
    
    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      mag_tmp[i] = mag[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, mag_tmp, mag);

    // Compensate gyroscope error
    gyr[0] -= GYRO_X_OFFSET;
    gyr[1] -= GYRO_Y_OFFSET;
    gyr[2] -= GYRO_Z_OFFSET;
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


void quaternion_to_matrix()
{
  rotation_matrix[0][0] = 1 - 2 * qua.y * qua.y - 2 * qua.z * qua.z;
  rotation_matrix[0][1] = 2 * qua.x * qua.y - 2 * qua.z * qua.w;
  rotation_matrix[0][2] = 2 * qua.x * qua.z + 2 * qua.y * qua.w;
  rotation_matrix[1][0] = 2 * qua.x * qua.y + 2 * qua.z * qua.w;
  rotation_matrix[1][1] = 1 - 2 * qua.x * qua.x - 2 * qua.z * qua.z;
  rotation_matrix[1][2] = 2 * qua.y * qua.z - 2 * qua.x * qua.w;
  rotation_matrix[2][0] = 2 * qua.x * qua.z - 2 * qua.y * qua.w;
  rotation_matrix[2][1] = 2 * qua.y * qua.z + 2 * qua.x * qua.w;
  rotation_matrix[2][2] = 1 - 2 * qua.x * qua.x - 2 * qua.y * qua.y;

  
}

void setup() {
  Serial.begin(1000000);
  while (!Serial) yield();
  //Wire.begin();
  Wire.setClock(4000000);
  sox.begin_I2C();

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );

  sox.setAccelDataRate(LSM6DS_RATE_52_HZ);

  sox.setGyroDataRate(LSM6DS_RATE_52_HZ);

  lis.begin_I2C();          // hardware I2C mode, can pass in address & alt Wire

  lis.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);

  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  lis.setDataRate(LIS3MDL_DATARATE_40_HZ);

  lis.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis.setIntThreshold(500);
  lis.configInterrupt(false, false, true, // enable z axis
                      true, // polarity
                      false, // don't latch
                      true); // enabled!

  data_init();
  time_former = micros();
}

void loop() {
  read_sensors();

  compensate_sensor_errors();

  time_now = micros();
  deltat = (float)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
  MadgwickQuaternionUpdate(acc[0], acc[1], acc[2],
                           gyr[0], gyr[1], gyr[2],
                           mag[0], mag[1], mag[2], deltat);
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];

  quaternion_to_matrix();

  
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.8; // 0.8 this used for compensating the angle between magenatic north and geographic north
  if (eul.yaw_e > PI) {
    eul.yaw_e -= 2 * PI;
  }
  /*
  Serial.print(eul.roll_e * 180.0 / PI);
  Serial.print(" ");
  Serial.print(eul.pitch_e * 180.0 / PI);
  Serial.print(" ");
  Serial.print(eul.yaw_e * 180.0 / PI);
  Serial.println(" ");
  */
  // Serial.println(deltat,7);
  // sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  delayMicroseconds(500);
}
