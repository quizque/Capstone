#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_FlashTransport.h>

// Create two MPU6050 instances
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

// Variables to store sensor data
float AccX1, AccY1, AccZ1;
float GyroX1, GyroY1, GyroZ1;
float AccX2, AccY2, AccZ2;
float GyroX2, GyroY2, GyroZ2;

// Define SPI flash pins
#define FLASH_CS   10
#define FLASH_MOSI 14
#define FLASH_MISO 15
#define FLASH_SCK  16

// Create an SPI flash transport instance
Adafruit_FlashTransport_SPI flashTransport(FLASH_CS);
Adafruit_SPIFlash flash(&flashTransport);

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin(); // Default SDA and SCL pins

  // Initialize the first MPU6050 (address 0x68)
  if (!mpu1.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip at address 0x68");
    while (1) {
      delay(10);
    }
  }
  mpu1.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu1.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_184_HZ);

  // Initialize the second MPU6050 (address 0x69)
  if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip at address 0x69");
    while (1) {
      delay(10);
    }
  }
  mpu2.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu2.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_184_HZ);

  // Initialize SPI flash
  Serial.println("Initializing SPI flash...");
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Flash chip initialized successfully.");
}

void loop() {
  sensors_event_t a1, g1, temp1;
  sensors_event_t a2, g2, temp2;

  // Get events from the first MPU6050
  mpu1.getEvent(&a1, &g1, &temp1);
  
  // Accelerometer values for first sensor
  AccX1 = a1.acceleration.x;
  AccY1 = a1.acceleration.y;
  AccZ1 = a1.acceleration.z;

  // Gyroscope values for first sensor
  GyroX1 = g1.gyro.x;
  GyroY1 = g1.gyro.y;
  GyroZ1 = g1.gyro.z;

  // Get events from the second MPU6050
  mpu2.getEvent(&a2, &g2, &temp2);
  
  // Accelerometer values for second sensor
  AccX2 = a2.acceleration.x;
  AccY2 = a2.acceleration.y;
  AccZ2 = a2.acceleration.z;

  // Gyroscope values for second sensor
  GyroX2 = g2.gyro.x;
  GyroY2 = g2.gyro.y;
  GyroZ2 = g2.gyro.z;

  // Print values from the first MPU6050
  Serial.print("Sensor 1 - X-Acceleration [in g] = ");
  Serial.print(AccX1 / 9.80665);
  Serial.print(", Y-Acceleration [in g] = ");
  Serial.print(AccY1 / 9.80665);
  Serial.print(", Z-Acceleration [in g] = ");
  Serial.println(AccZ1 / 9.80665);

  // Print values from the second MPU6050
  Serial.print("Sensor 2 - X-Acceleration [in g] = ");
  Serial.print(AccX2 / 9.80665);
  Serial.print(", Y-Acceleration [in g] = ");
  Serial.print(AccY2 / 9.80665);
  Serial.print(", Z-Acceleration [in g] = ");
  Serial.println(AccZ2 / 9.80665);
  
  // Write data to flash memory
  uint32_t addr = 0;
  flash.writeBuffer(addr, (uint8_t *)&AccX1, sizeof(AccX1));
  addr += sizeof(AccX1);
  flash.writeBuffer(addr, (uint8_t *)&AccY1, sizeof(AccY1));
  addr += sizeof(AccY1);
  flash.writeBuffer(addr, (uint8_t *)&AccZ1, sizeof(AccZ1));
  addr += sizeof(AccZ1);
  flash.writeBuffer(addr, (uint8_t *)&GyroX1, sizeof(GyroX1));
  addr += sizeof(GyroX1);
  flash.writeBuffer(addr, (uint8_t *)&GyroY1, sizeof(GyroY1));
  addr += sizeof(GyroY1);
  flash.writeBuffer(addr, (uint8_t *)&GyroZ1, sizeof(GyroZ1));
  addr += sizeof(GyroZ1);
  flash.writeBuffer(addr, (uint8_t *)&AccX2, sizeof(AccX2));
  addr += sizeof(AccX2);
  flash.writeBuffer(addr, (uint8_t *)&AccY2, sizeof(AccY2));
  addr += sizeof(AccY2);
  flash.writeBuffer(addr, (uint8_t *)&AccZ2, sizeof(AccZ2));
  addr += sizeof(AccZ2);
  flash.writeBuffer(addr, (uint8_t *)&GyroX2, sizeof(GyroX2));
  addr += sizeof(GyroX2);
  flash.writeBuffer(addr, (uint8_t *)&GyroY2, sizeof(GyroY2));
  addr += sizeof(GyroY2);
  flash.writeBuffer(addr, (uint8_t *)&GyroZ2, sizeof(GyroZ2));

  delay(500);
}