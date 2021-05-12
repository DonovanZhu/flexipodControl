#include <Adafruit_LSM6DSOX.h> //ref:https://github.com/adafruit/Adafruit_LSM6DS/blob/master/Adafruit_LSM6DS.h
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>

Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mage;
sensors_event_t temp;


void read_sensors() {
  sox.getEvent(&accel, &gyro, &temp);
  lis.getEvent(&mage);
//  acc[0] = accel.acceleration.x;
//  acc[1] = accel.acceleration.y;
//  acc[2] = accel.acceleration.z;
}

int t1;

void setup() {
  Serial.begin(2000000);
  while (!Serial) yield();
  
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
//  lis.setIntThreshold(500);
//  lis.configInterrupt(false, false, true, // enable z axis
//                      true, // polarity
//                      false, // don't latch
//                      true); // enabled!

  t1 = micros();
}

void loop() {
  int dt = micros() - t1;
  t1 = micros();
  
  read_sensors();
  char buf [256];
  int n;
  
  n = sprintf(buf,"%f %f %f %f %f %f %f %f %f %f\n",(float)dt,
              accel.acceleration.x,accel.acceleration.y,accel.acceleration.z,
              gyro.gyro.x,gyro.gyro.y,gyro.gyro.z,
              mage.magnetic.x,mage.magnetic.y,mage.magnetic.z);
  Serial.write(buf,n);
//  Serial.flush();
//  delayMicroseconds(1000);
  
}
