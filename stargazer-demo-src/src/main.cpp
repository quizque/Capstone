#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
// #include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <SPIMemory.h>
#include <RadioLib.h>
#include <SD.h>
#include <LPS.h>
#include <LSM6.h>
#include <LIS3MDL.h>

#include "stargazer_constants.h"
#include "data_header.h"

/////////////////////////////////////////////
// Configuration

// Enable/disable transmitter mode and select
// the type of transmitter

// #define TRANSMITTER 1
// #define MEASURE_STORE_RATE 1

#define USE_LORA
// #define USING_SX1262

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

/////////////////////////////////////////////
// Variables definitions

// Radio config
#ifdef USING_SX1262
SX1262
#else
SX1276
#endif
radio = new Module(LORA_CS, LORA_IRQ, LORA_RST);

// Flash memory
SPIFlash flash(FLASH_CS);

// Sensors
LPS ps;
LSM6 imu;
LIS3MDL mag;

// Enabled devices
// This is automatically set by the program
bool gps_enabled = false;
bool pressure_enabled = false;
bool imu_enabled = false;
bool mag_enabled = false;
bool high_imu = false;

// Is high data rate enabled
bool override_enabled = false;

// Transmission state
int transmissionState = RADIOLIB_ERR_NONE;
uint8_t radio_round_robin = 0;

void setup()
{
  ///
  /// Setup USB serial
  ///

  USBSerial.begin(115200);
  delay(2000); // Wait for USB to connect, mostly for debugging
  USBSerial.println("[INFO] Serial started.");

  ///
  /// Setup LED
  ///

  // Enable the PWM component for the LED
  ledcSetup(0, 5000, 8);
  ledcAttachPin(LED_STATUS, 0);

  USBSerial.println("[INFO] LED started.");

  ///
  /// SPI Setup
  ///

  pinMode(FLASH_CS, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  ////////////////////////////////////////////////////////////////

  ///
  /// Setup I2C
  ///

  Wire.begin();
  // i2c 100kHz
  Wire.setClock(100000);
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
    // Enable medium performance mode
    mag.writeReg(LIS3MDL::CTRL_REG2, 0x20);
  }

  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE)
  {
    USBSerial.print(F("[ERROR] Radio not found, code:"));
    USBSerial.println(state);
    // If the radio is not found, we will just hang here
    while (1)
      ;
  }
  else
  {

    USBSerial.println("[INFO] Radio found.");
  }

  // Set frequency, bandwidth, spreading factor, coding rate, etc.
  radio.setFrequency(912.0);
  radio.setBandwidth(500.0);
  radio.setSpreadingFactor(6);
  radio.setCodingRate(7);
  radio.setPreambleLength(8);
  radio.setOutputPower(RADIOLIB_SX1278_MAX_POWER);
  radio.setCRC(true);
}

unsigned long lastTime = 0; // Calculate loop time

float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

// Complementary filter constant, controls how "reactive"
// the filter is to changes in the gyroscope data
const float alpha = 0.8;

// Quaternion values
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

// Function to normalize a quaternion
// Simply remaps all values to have a magnitude of 1
// (i.e. all values end up on the unit sphere)
void normalizeQuaternion(float &qw, float &qx, float &qy, float &qz)
{
  float norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;
}

// Function to convert a quaternion to Euler angles
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
// https://www.nxp.com/docs/en/application-note/AN3461.pdf
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

// Recompute the quanternion by biasing it with the magnetometer data
// Only affects yaw since pitch/roll are already solvable by the IMU.
// NOTE: This is a poor implementation, ideally, this should only be used on startup
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/Heading-Calculation/td-p/88979/page/2
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

// Get the sign of a float
float get_sign(float x)
{
  if (x < 0)
  {
    return -1;
  }
  return 1;
}

void loop()
{

#ifdef TRANSMITTER

  // Read the time since the last loop
  float dt = (micros() - lastTime) / 1000000.0;

  lastTime = micros();

  // Read the sensors
  imu.read();
  mag.read();

  // Read and convert the accelerometer data
  float ax = (float)imu.a.x * 0.122 / 1000.0;
  float ay = (float)imu.a.y * 0.122 / 1000.0;
  float az = (float)imu.a.z * 0.122 / 1000.0;

  // Calculate the g-force
  float g_force = sqrt(ax * ax + ay * ay + az * az) * get_sign(az);

  // Read and convert the gyroscope data (in rad/s, for the complementary filter)
  float gx_rad = (imu.g.x * 35.0 / 1000.0) * (PI / 180);
  float gy_rad = (imu.g.y * 35.0 / 1000.0) * (PI / 180);
  float gz_rad = (imu.g.z * 35.0 / 1000.0) * (PI / 180);

  // Read and convert the magnetometer data
  float mx = (((float)mag.m.x / 3421.0) - -0.77) / 0.38;
  float my = (((float)mag.m.y / 3421.0) - 1.16) / 0.38;
  float mz = (((float)mag.m.z / 3421.0) - -2.04) / 0.43;

  ///////////////////////////////////////////////
  // Quaternion Complementary Filter
  //
  // NOTE: This is pretty thrown together from basic knowledge
  //       and some online resources. It is not perfect and doesn't
  //       work along the X/Y axis correctly.
  //
  // This filter works by "correcting" its self with a gravity vector
  // and then updating at a high speed with the gyroscope data. The
  // magnetometer data is used to correct the yaw angle (but doesn't
  // work as intended, kind of).

  // Normalize the magnetometer data
  float norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm;
  ay /= norm;
  az /= norm;

  // Convert from quaternion to gravity vector
  float vx = 2 * (q1 * q3 - q0 * q2);
  float vy = 2 * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // Compute the error between the accelerometer vector and the gravity vector
  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  // Apply the error to the gyroscope data
  gx_rad += alpha * ex;
  gy_rad += alpha * ey;
  gz_rad += alpha * ez;

  // Calculate the quaternion derivative
  float q0_dot = 0.5 * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
  float q1_dot = 0.5 * (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
  float q2_dot = 0.5 * (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
  float q3_dot = 0.5 * (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

  // Integrate the quaternion derivative
  q0 += q0_dot * dt;
  q1 += q1_dot * dt;
  q2 += q2_dot * dt;
  q3 += q3_dot * dt;

  // Normalize the quaternion
  // The derivative integration can cause some values to go out of bounds
  normalizeQuaternion(q0, q1, q2, q3);

  // Assemble the data to be transmitted
  TransmitData td;

  td.q0 = q0;
  td.q1 = q1;
  td.q2 = q2;
  td.q3 = q3;
  td.pressure = ps.readPressureMillibars();
  td.gforce = g_force;
  td.temperature = ps.readTemperatureC();

  // Transmit the data
  int state = radio.transmit((uint8_t *)&td, sizeof(TransmitData));
  if (state != RADIOLIB_ERR_NONE)
  {
    USBSerial.print(F("[ERROR] Failed to transmit, code: "));
    USBSerial.println(state);
  }

  // Print the data to the serial monitor for when the board is connected to the computer
  USBSerial.printf("%f,%f,%f,%f,%f,%f,%f\n", td.q0, td.q1, td.q2, td.q3, td.pressure, td.gforce, td.temperature);

  // Required due to the radio library having a lot of weird issues
  delay(10);

#else

  // Setup the data to be received
  uint8_t data[sizeof(TransmitData)];
  int state = radio.receive(data, sizeof(TransmitData));

  if (state == RADIOLIB_ERR_NONE)
  {
    // Cast bytes to struct so we can access the data
    TransmitData *td = (TransmitData *)data;

    // Print the data to the serial monitor for when the board is connected to the computer
    USBSerial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n", td->q0, td->q1, td->q2, td->q3, td->pressure, td->gforce, td->temperature, radio.getRSSI(), radio.getSNR());
  }

#endif
}