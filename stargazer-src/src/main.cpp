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

#define TRANSMITTER 1
// #define MEASURE_STORE_RATE 1

#define USE_LORA
// #define USING_SX1262

#define FLASH_MARKER_PRIMARY 0x8
#define FLASH_MARKER_SECONDARY 0xC

// Data store rate
#define DATA_GPS_RATE_NO_LOCK 2000
#define DATA_GPS_RATE_LOCK 220

#define DATA_PRESSURE_RATE 200

#define DATA_IMU_RATE 200

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

uint32_t current_flash_address = 0x0;
const char *flash_marker = "STARGAZE";
uint8_t flashData[4096];
void refresh_write(uint32_t address, uint8_t *data, uint32_t length)
{
  uint32_t rounded_4k_addr = ((address / 4096) * 4096);
  if (!flash.readByteArray(rounded_4k_addr, &flashData[0], 4096))
  {
    USBSerial.println("[INFO] Flash read failed, halting.");
    led_panic();
  }

  if (!flash.eraseSector(rounded_4k_addr))
  {
    USBSerial.println("[INFO] Flash erase failed, halting.");
    led_panic();
  }

  memcpy(&flashData[address - rounded_4k_addr], data, length);

  if (!flash.writeByteArray(rounded_4k_addr, &flashData[0], 4096, true))
  {
    USBSerial.println("[INFO] Flash write failed, halting.");
    led_panic();
  }
}

void init_flash()
{
  uint8_t flashData[9];
  flash.readByteArray(0x0, &flashData[0], 8);

  if (memcmp(flashData, flash_marker, 8) != 0)
  {
    USBSerial.println("[WARNING] Flash not initialized, setting up...");
    flash.eraseSector(0x0);
    flash.writeByteArray(0x0, (uint8_t *)flash_marker, 8, true);

    flash.writeULong(FLASH_MARKER_PRIMARY, 16, true);
    flash.writeULong(FLASH_MARKER_SECONDARY, 16, true);
    USBSerial.println("[INFO] Flash initialized.");
  }
  else
  {
    USBSerial.println("[INFO] Flash already initialized.");
  }

  USBSerial.println("[INFO] Getting current flash address.");
  uint32_t primary_marker = 0x0;
  uint32_t secondary_marker = 0x0;
  primary_marker = flash.readULong(FLASH_MARKER_PRIMARY);
  secondary_marker = flash.readULong(FLASH_MARKER_SECONDARY);

  if (primary_marker != secondary_marker)
  {
    USBSerial.println("[ERROR] Flash markers do not match, using earliest.");
    current_flash_address = primary_marker;
  }
  else
  {
    current_flash_address = secondary_marker;
  }

  USBSerial.print("[INFO] Current flash address: ");
  USBSerial.println(current_flash_address);
}

unsigned long lastWrite = 0;
unsigned long totalWriteBytes = 0;
void write_flash(uint8_t *data, uint32_t length)
{
#ifdef MEASURE_STORE_RATE
  totalWriteBytes += length;
  if (millis() - lastWrite > 5000)
  {
    USBSerial.print("[INFO] Write rate: ");
    USBSerial.print(totalWriteBytes / 5);
    USBSerial.println(" bytes/s");
    lastWrite = millis();
    totalWriteBytes = 0;
  }
#else
  if (flash.writeByteArray(current_flash_address, data, length, true))
  {
    USBSerial.println("[INFO] Flash write success.");
    current_flash_address += length;
  }
  else
  {
    USBSerial.println("[INFO] Flash write failed, erasing.");
    // Refresh sector above and below (check to s)
    // refresh_sector(current_flash_address);
    // refresh_sector(current_flash_address + 4096);

    USBSerial.printf("[INFO] Flash erase addr %d.", current_flash_address);
    if (flash.writeByteArray(current_flash_address, data, length, true))
    {
      USBSerial.println("[INFO] Flash write success.");
      current_flash_address += length;
    }
    else
    {
      USBSerial.println("[ERROR] Flash write failed, HALTING.");
      // debug info
      USBSerial.printf("Current flash address: %d\n", current_flash_address);
      USBSerial.printf("Data: %d %d %d %d\n", data[0], data[1], data[2], data[3]);
      USBSerial.printf("Data: %d %d %d %d\n", data[4], data[5], data[6], data[7]);
      USBSerial.printf("Length: %d\n", length);
      led_panic();
    }
  }

  bool r1 = flash.writeULong(FLASH_MARKER_PRIMARY, current_flash_address, true);
  bool r2 = flash.writeULong(FLASH_MARKER_SECONDARY, current_flash_address, true);

  USBSerial.printf("[INFO] Flash markers -> %d %d\n", r1, r2);

  uint8_t readData1[32];
  if (flash.readByteArray(0x0, &readData1[0], 32))
  {
    USBSerial.println("[INFO] Flash read success.");
    for (int i = 0; i < 32; i++)
    {
      USBSerial.print(readData1[i], HEX);
      USBSerial.print(" ");
    }
    USBSerial.println();
  }
  else
  {
    USBSerial.println("[ERROR] Flash read failed.");
  }

#endif
}

void setup()
{
  ///
  /// Setup USB serial
  ///

  USBSerial.begin(115200);
  delay(5000);
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

  digitalWrite(FLASH_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  SPI.begin();

  USBSerial.println("[INFO] SPI started.");

  // Check for flash
  flash.begin();
  USBSerial.println("[INFO] Checking for flash.");
  uint32_t JEDEC = flash.getJEDECID();
  if (0xef4018 != JEDEC)
  {
    USBSerial.println("[ERROR] Flash not found, halting.");
    led_panic();
  }
  USBSerial.println("[INFO] Flash found.");

  // Copy first 4k, erase, write back
  USBSerial.println("[INFO] Performing flash workaround.");

  //////////////////////////////////////////////////////////////////////////////////////////

  uint8_t readData1[32];
  if (flash.readByteArray(0x0, &readData1[0], 32))
  {
    USBSerial.println("[INFO] Flash read success.");
    for (int i = 0; i < 32; i++)
    {
      USBSerial.print(readData1[i], HEX);
      USBSerial.print(" ");
    }
    USBSerial.println();
  }
  else
  {
    USBSerial.println("[ERROR] Flash read failed.");
  }
  //////////////////////////////////////////////////////////////////////////////////////////

  // Check for radio
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE)
  {
    USBSerial.print(F("[ERROR] Radio not found, code:"));
    USBSerial.println(state);
#ifdef USE_LORA
    USBSerial.println("[ERROR] halting.");
    led_panic();
#endif
    USBSerial.println("[WARN] Radio not found, continuing.");
  }
  else
  {
    // #ifdef USING_SX1262 & !TRANSMITTER
    //     radio.setDio1Action(setFlag);
    // #else
    //     radio.setDio0Action(setFlag);
    // #endif

    USBSerial.println("[INFO] Radio found.");
  }

  // Check for SD card
  if (!SD.begin(SD_CS))
  {
    USBSerial.println("[WARNING] SD card not found, continuing.");
  }
  else
  {
    USBSerial.println("[INFO] SD card found.");
  };

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
    imu.writeReg(LSM6::CTRL1_XL, 0x84);
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
    mag.writeReg(LIS3MDL::CTRL_REG2, 0x60);
  }

  ///
  /// GPS Setup
  ///

  if (gnss.begin() == false)
  {
    USBSerial.println("[WARNING] GPS not found, continuing.");
  }
  else
  {
    USBSerial.println("[INFO] GPS found.");
    gps_enabled = true;
  }

  USBSerial.println("[INFO] Initialization complete.");

  ///
  /// Device setup complete
  ///

#ifdef USE_LORA
  radio.setFrequency(919.0);
  radio.setBandwidth(512.0);
  radio.setSpreadingFactor(10);
  radio.setCodingRate(5);
  radio.setCRC(true);
#endif

  gnss.setI2COutput(COM_TYPE_UBX);
  gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  gnss.setNavigationFrequency(5);

  // flash.eraseBlock64K(0x0);
  // if (flash.writeByteArray(0x0, &flashData[0], 63, true))
  // {
  //   USBSerial.println("[INFO] Flash write success.");
  // }
  // else
  // {
  //   USBSerial.println("[ERROR] Flash write failed.");
  // }

  // Print first 16 bytes of flash
  init_flash();

  uint8_t readData[32];
  if (flash.readByteArray(0x0, &readData[0], 32))
  {
    USBSerial.println("[INFO] Flash read success.");
    for (int i = 0; i < 32; i++)
    {
      USBSerial.print(readData[i], HEX);
      USBSerial.print(" ");
    }
    USBSerial.println();
  }
  else
  {
    USBSerial.println("[ERROR] Flash read failed.");
  }
}

unsigned long lastGPS = 0;
bool gpsLock = false;

unsigned long lastPressure = 0;

unsigned long lastIMU = 0;

unsigned long lastMag = 0;

unsigned long lastTime = 0; // Calculate loop time

void loop()
{
  // USBSerial.printf("Loop time: %d\n", millis() - lastTime);
  // lastTime = millis();

  unsigned long currentMillis = millis();

  //////////////////////////////////////////
  /// GPS Data
  ///

  if (!gpsLock)
  {
    led_fade_update();
  }

  if (!gpsLock && currentMillis - lastGPS > DATA_GPS_RATE_NO_LOCK)
  {

    if (gnss.getFixType() != 0)
    {
      USBSerial.println("LOCK OBTAINED");
      ledSetHigh();
      gpsLock = true;
    }
    else
    {
      USBSerial.println("NO LOCK OBTAINED");

      gpsLock = false;
    }

    lastGPS = currentMillis;
  }

  if (gpsLock && currentMillis - lastGPS > DATA_GPS_RATE_LOCK)
  {
    GPSData gpsData;
    gpsData.header.data_size = sizeof(GPSData);
    gpsData.header.data_type = DATA_TYPE_GPS;

    gpsData.year = gnss.getYear();
    gpsData.month = gnss.getMonth();
    gpsData.day = gnss.getDay();

    gpsData.hour = gnss.getHour();
    gpsData.minute = gnss.getMinute();
    gpsData.second = gnss.getSecond();

    gpsData.latitude = gnss.getLatitude();
    gpsData.longitude = gnss.getLongitude();
    gpsData.altitude = gnss.getAltitude();
    gpsData.satellites = gnss.getSIV();
    gpsData.fix = gnss.getFixType();

    gpsData.speed = gnss.getGroundSpeed();
    gpsData.heading = gnss.getHeading();

    if (gpsData.fix == 0)
    {
      USBSerial.println("NO LOCK OBTAINED");
      gpsLock = false;
    }
    else
    {
      USBSerial.println("LOCK OBTAINED");
      USBSerial.printf("Lat: %d Long: %d Alt: %d Speed: %d Sats: %d Fix: %d\n", gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.speed, gpsData.satellites, gpsData.fix);
      write_flash((uint8_t *)&gpsData, sizeof(GPSData));
      lastGPS = currentMillis;
    }
  }

  //////////////////////////////////////////
  /// Pressure Data
  ///

  if (currentMillis - lastPressure > DATA_PRESSURE_RATE && pressure_enabled)
  {
    float pressure = ps.readPressureMillibars();
    float temperature = ps.readTemperatureC();

    PressureData pressureData;
    pressureData.header.data_size = sizeof(PressureData);
    pressureData.header.data_type = DATA_TYPE_PRESSURE;

    pressureData.pressure = pressure;
    pressureData.temperature = temperature;

    write_flash((uint8_t *)&pressureData, sizeof(PressureData));

    lastPressure = currentMillis;
  }

  //////////////////////////////////////////
  /// IMU Data
  ///

  if (currentMillis - lastIMU > DATA_IMU_RATE && imu_enabled)
  {
    imu.read();

    IMUData imuData;
    imuData.header.data_size = sizeof(IMUData);
    imuData.header.data_type = DATA_TYPE_IMU;

    imuData.acc_x = imu.a.x * 0.488;
    imuData.acc_y = imu.a.y * 0.488;
    imuData.acc_z = imu.a.z * 0.488;

    imuData.gyro_x = imu.g.x * 35.0;
    imuData.gyro_y = imu.g.y * 35.0;
    imuData.gyro_z = imu.g.z * 35.0;

    // USBSerial.printf("Accel: %d %d %d Gyro: %d %d %d\n", imuData.acc_x, imuData.acc_y, imuData.acc_z, imuData.gyro_x, imuData.gyro_y, imuData.gyro_z);

    write_flash((uint8_t *)&imuData, sizeof(IMUData));

    lastIMU = currentMillis;
  }

  //////////////////////////////////////////
  /// Mag Data
  ///

  if (currentMillis - lastMag > DATA_IMU_RATE && mag_enabled)
  {
    mag.read();

    MagData magData;
    magData.header.data_size = sizeof(MagData);
    magData.header.data_type = DATA_TYPE_MAG;

    magData.mag_x = (float)mag.m.x / 1711.0;
    magData.mag_y = (float)mag.m.y / 1711.0;
    magData.mag_z = (float)mag.m.z / 1711.0;

    // USBSerial.printf("Mag: %f %f %f\n", magData.mag_x, magData.mag_y, magData.mag_z);

    write_flash((uint8_t *)&magData, sizeof(MagData));

    lastMag = currentMillis;
  }
}