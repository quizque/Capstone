#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create two MPU6050 instances
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

// Variables to store sensor data
float AccX1, AccY1, AccZ1;
float GyroX1, GyroY1, GyroZ1;
float AccX2, AccY2, AccZ2;
float GyroX2, GyroY2, GyroZ2;

void setup() {
  Serial.begin(115200);
  // Initialize I2C communication (Pins are Default Wires)
  Wire.begin();

  // Initialize the first MPU
  if (!mpu1.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip at address 0x68");
    while (1) {
      delay(10);
    }
  }
  mpu1.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu1.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_184_HZ);

  // Initialize the second MPU
  if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip at address 0x69");
    while (1) {
      delay(10);
    }
  }
  mpu2.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu2.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_184_HZ);
}

void loop() {
  sensors_event_t a1, g1, temp1;
  sensors_event_t a2, g2, temp2;

  mpu1.getEvent(&a1, &g1, &temp1);
  
  // Accelerometer values for first sensor
  AccX1 = a1.acceleration.x;
  AccY1 = a1.acceleration.y;
  AccZ1 = a1.acceleration.z;

  // Gyroscope values for first sensor
  GyroX1 = g1.gyro.x;
  GyroY1 = g1.gyro.y;
  GyroZ1 = g1.gyro.z;

  mpu2.getEvent(&a2, &g2, &temp2);
  
  // Accelerometer values for second sensor
  AccX2 = a2.acceleration.x;
  AccY2 = a2.acceleration.y;
  AccZ2 = a2.acceleration.z;

  // Gyroscope values for second sensor
  GyroX2 = g2.gyro.x;
  GyroY2 = g2.gyro.y;
  GyroZ2 = g2.gyro.z;

  // Print values from the first MPU
  Serial.print("Sensor 1 - X-Acceleration [in g] = ");
  Serial.print(AccX1 / 9.80665);
  Serial.print(", Y-Acceleration [in g] = ");
  Serial.print(AccY1 / 9.80665);
  Serial.print(", Z-Acceleration [in g] = ");
  Serial.println(AccZ1 / 9.80665);

  // Print values from the second MPU
  Serial.print("Sensor 2 - X-Acceleration [in g] = ");
  Serial.print(AccX2 / 9.80665);
  Serial.print(", Y-Acceleration [in g] = ");
  Serial.print(AccY2 / 9.80665);
  Serial.print(", Z-Acceleration [in g] = ");
  Serial.println(AccZ2 / 9.80665);
  
  delay(500);
}