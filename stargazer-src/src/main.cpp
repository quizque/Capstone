#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <SPIMemory.h>
#include <RadioLib.h>
#include <SD.h>
#include <LPS.h>
#include <LSM6.h>
#include <LIS3MDL.h>

#include "stargazer_constants.h"
#include "data_header.h"

// #define TRANSMITTER 1
// #define MEASURE_STORE_RATE 1

#define USE_LORA
#define USING_SX1262

// Data store rate
#define DATA_GPS_RATE_NO_LOCK 2000
#define DATA_GPS_RATE_LOCK 250

#define DATA_PRESSURE_RATE 200

#define DATA_IMU_RATE 200

#define DATA_MAG_RATE 200

#define DATA_RADIO_RATE 300

// If the acc goes above this value, the rocket has launched
// and all devices will start recording at a higher data rate
#define ACC_OVERDRIVE_WRITE 2.5

// Radio config

#ifdef USING_SX1262
SX1262
#else
SX1276
#endif
radio = new Module(LORA_CS, LORA_IRQ, LORA_RST);

SPIFlash flash(FLASH_CS);

LPS ps;
LSM6 imu;
LIS3MDL mag;

SFE_UBLOX_GNSS gnss;

// Enabled devices
bool gps_enabled = false;
bool pressure_enabled = false;
bool imu_enabled = false;
bool mag_enabled = false;
bool high_imu = false;

bool override_enabled = false;

volatile bool operationDone = false;
int transmissionState = RADIOLIB_ERR_NONE;
uint8_t radio_round_robin = 0;

float init_height = 0;

short unsigned int led_current_pwm = 0;
unsigned long int led_current_time = 0;
bool led_up = true;
void led_fade_update()
{
  if (millis() - led_current_time < 1)
  {
    return;
  }
  if (led_up)
  {
    led_current_pwm += 1;
    if (led_current_pwm >= 255)
    {
      led_up = false;
    }
  }
  else
  {
    led_current_pwm -= 1;
    if (led_current_pwm <= 0)
    {
      led_up = true;
    }
  }

  led_current_time = millis();
  ledcWrite(0, led_current_pwm);
}

void ledSetHigh()
{
  ledcWrite(0, 255);
}

void ledSetLow()
{
  ledcWrite(0, 0);
}

void led_panic()
{
  while (1)
  {
    ledcWrite(0, 0);
    delay(100);
    ledcWrite(0, 255);
    delay(100);
  }
}

void setup()
{
  ///
  /// Setup USB serial
  ///

  USBSerial.begin(115200);
  delay(2000);
  USBSerial.println("[INFO] Serial started.");

  ///
  /// Setup LED
  ///

  // pinMode(LED_STATUS, OUTPUT);
  // analogWrite(LED_STATUS, 30);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(LED_STATUS, 0);

  USBSerial.println("[INFO] LED started.");

  ///
  /// SPI Setup
  ///

  pinMode(FLASH_CS, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  //   //////////////////////////////////////////////////////////////////////////////////////////

  ///
  /// Setup I2C
  ///

  Wire.begin();
  USBSerial.println("[INFO] I2C started.");

  // Check Barometer
  if (!ps.init())
  {
    USBSerial.println("[WARNING] Barometer not found, continuing.");
  }
  else
  {
    USBSerial.println("[INFO] Barometer found.");
    pressure_enabled = true;
    ps.enableDefault();
  }

  // Check gyro/acc
  if (!imu.init())
  {
    USBSerial.println("[WARNING] Gyro/acc not found, continuing.");
  }
  else
  {
    USBSerial.println("[INFO] IMU found.");
    imu_enabled = true;

    imu.enableDefault();
    imu.writeReg(LSM6::CTRL1_XL, 0x88);
    imu.writeReg(LSM6::CTRL2_G, 0x88);
  }

  // Check Magnetometer
  if (!mag.init())
  {
    USBSerial.println("[WARNING] Magnetometer not found, continuing.");
  }
  else
  {
    USBSerial.println("[INFO] Magnetometer found.");
    mag_enabled = true;

    mag.enableDefault();
    mag.writeReg(LIS3MDL::CTRL_REG2, 0x20);
  }
}

unsigned long lastTime = 0; // Calculate loop time

float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;
const float alpha = 0.98;

float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

void normalizeQuaternion(float &qw, float &qx, float &qy, float &qz)
{
  float norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;
}

void quaternionToEuler(float qw, float qx, float qy, float qz, float &yaw, float &pitch, float &roll)
{
  // Roll (x-axis rotation)
  float sinr_cosp = 2 * (qw * qx + qy * qz);
  float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  float sinp = 2 * (qw * qy - qz * qx);
  if (abs(sinp) >= 1)
    pitch = copysign(PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // Yaw (z-axis rotation)
  float siny_cosp = 2 * (qw * qz + qx * qy);
  float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  yaw = atan2(siny_cosp, cosy_cosp);

  // Convert radians to degrees
  yaw = yaw * 180 / PI;
  pitch = pitch * 180 / PI;
  roll = roll * 180 / PI;
}

// Function to compute the yaw from magnetometer data
float computeMagYaw(float mx, float my, float mz, float q0, float q1, float q2, float q3)
{
  // Normalize magnetometer data
  float norm = sqrt(mx * mx + my * my + mz * mz);
  mx /= norm;
  my /= norm;
  mz /= norm;

  // Compute the Earth's magnetic field vector in the body frame
  float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
  float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));

  // Calculate yaw
  return atan2(hy, hx);
}

void loop()
{

  // // USBSerial.printf("Loop time: %d\n", millis() - lastTime);
  // // lastTime = millis();

  // 500 Hz
  while (micros() - lastTime < 2000)
  {
  }

  float dt = (micros() - lastTime) / 1000000.0;

  lastTime = micros();

  imu.read();

  float ax = (float)imu.a.x * 0.122 / 1000.0;
  float ay = (float)imu.a.y * 0.122 / 1000.0;
  float az = (float)imu.a.z * 0.122 / 1000.0;

  float gx_rad = (imu.g.x * 35.0 / 1000.0) * (PI / 180);
  float gy_rad = (imu.g.y * 35.0 / 1000.0) * (PI / 180);
  float gz_rad = (imu.g.z * 35.0 / 1000.0) * (PI / 180);

  float mx = (float)mag.m.x / 3421.0;
  float my = (float)mag.m.y / 3421.0;
  float mz = (float)mag.m.z / 3421.0;

  float norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm;
  ay /= norm;
  az /= norm;

  float vx = 2 * (q1 * q3 - q0 * q2);
  float vy = 2 * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  gx_rad += alpha * ex;
  gy_rad += alpha * ey;
  gz_rad += alpha * ez;

  float q0_dot = 0.5 * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
  float q1_dot = 0.5 * (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
  float q2_dot = 0.5 * (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
  float q3_dot = 0.5 * (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

  q0 += q0_dot * dt;
  q1 += q1_dot * dt;
  q2 += q2_dot * dt;
  q3 += q3_dot * dt;

  normalizeQuaternion(q0, q1, q2, q3);

  float mag_yaw = computeMagYaw(mx, my, mz, q0, q1, q2, q3) * 180 / PI;

  float yaw_gyro = 2 * (q0 * q3 + q1 * q2);
  yaw_gyro = atan2(yaw_gyro, 1 - 2 * (q2 * q2 + q3 * q3)) * 180 / PI;
  float yaw = alpha * yaw_gyro + (1 - alpha) * mag_yaw;

  float pitch, roll;
  quaternionToEuler(q0, q1, q2, q3, yaw, pitch, roll);

  // Print the Euler angles
  USBSerial.printf("%f,%f,%f,%f\n", q0, q1, q2, q3);

  // //////////////////////////////////////////
  // /// Mag Data
  // ///

  // if ((currentMillis - lastMag > DATA_MAG_RATE && mag_enabled && lastOverdrive > currentMillis) ||
  //     (currentMillis - lastMag > DATA_MAG_RATE * 3 && mag_enabled && lastOverdrive < currentMillis))
  // {
  //   mag.read();

  //   magData.header.data_size = sizeof(MagData);
  //   magData.header.data_type = DATA_TYPE_MAG;
  //   magData.header.timestamp = millis();

  //   magData.mag_x = (float)mag.m.x / 1711.0;
  //   magData.mag_y = (float)mag.m.y / 1711.0;
  //   magData.mag_z = (float)mag.m.z / 1711.0;

  //   // USBSerial.printf("Mag: %f %f %f\n", magData.mag_x, magData.mag_y, magData.mag_z);

  //   write_flash((uint8_t *)&magData, sizeof(MagData));

  //   lastMag = currentMillis;
  // }
}