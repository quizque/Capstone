#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

// Complementary filter parameter
const float alpha = 0.98;

// Time
unsigned long currentTime, previousTime;
float elapsedTime;

// Angles
float angleGyroX = 0, angleGyroY = 0, angleGyroZ = 0; // Gyroscope angle change
float angleAccX, angleAccY; // Accelerometer angle
float angleX = 0, angleY = 0, angleZ = 0; // Final angles (roll, pitch, yaw)

void setup()
{
  Serial.begin(115200);
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();

  if (!imu.init())
  {
    while (1){
      Serial.println("Failed to detect and initialize IMU!");
    }
  }
  imu.enableDefault();

  previousTime = micros();
}

void loop()
{
  imu.read();
  
  // Current time actual measurement
  currentTime = micros();
  // Elapsed time in seconds
  elapsedTime = (currentTime - previousTime) / 1000000.0; 
  previousTime = currentTime;

  // Convert accelerometer values to m/s²
  float accX = (float)imu.a.x*0.061/1000.0*9.80665;
  float accY = (float)imu.a.y*0.061/1000.0*9.80665;
  float accZ = (float)imu.a.z*0.061/1000.0*9.80665;

  // Convert gyroscope values to degrees/s
  float gyroX = (float)imu.g.x*8.75/1000.0;
  float gyroY = (float)imu.g.y*8.75/1000.0;
  float gyroZ = (float)imu.g.z*8.75/1000.0;

  // Calculate the angle of the accelerometer
  angleAccX = atan2(accY, accZ) * 180 / PI;
  angleAccY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  angleGyroX += gyroX * elapsedTime;
  angleGyroY += gyroY * elapsedTime;
  angleGyroZ += gyroZ * elapsedTime; // Calculate yaw change

  // Complementary filter - combine accelerometer and gyro angle values
  angleX = alpha * (angleX + gyroX * elapsedTime) + (1.0 - alpha) * angleAccX;
  angleY = alpha * (angleY + gyroY * elapsedTime) + (1.0 - alpha) * angleAccY;
  // For yaw, we only rely on gyroscope integration as there's no accelerometer reference
  angleZ = angleGyroZ; // Direct integration without correction for drift

  // Print the values to the Serial Monitor
  Serial.print("Roll: ");
  Serial.print(angleX);
  Serial.print(" Pitch: ");
  Serial.print(angleY);
  Serial.print(" Yaw: ");
  Serial.println(angleZ);

  //delay(1);
}
