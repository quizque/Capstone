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

// #include <Geometry.h>
// #include <BasicLinearAlgebra.h>

#include "stargazer_constants.h"
#include "data_header.h"
#include "MagCalibrate.hpp"

// #define TRANSMITTER 1
//   #define MEASURE_STORE_RATE 1

#define USE_LORA
#define USING_SX1262

// Data store rate
#define DATA_GPS_RATE_NO_LOCK 2000
#define DATA_GPS_RATE_LOCK 220

#define DATA_PRESSURE_RATE 200

#define DATA_IMU_RATE 200

#define DATA_MAG_RATE 200

#define DATA_RADIO_RATE 500

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

const float hard_iron[3] = {
    -32.71, -7.94, -10.54};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
    {0.992, 0.007, 0.011},
    {0.007, 1.011, -0.024},
    {0.011, -0.024, 0.998}};

volatile bool receivedFlag = false;
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void)
{
  // we got a packet, set the flag
  receivedFlag = true;
}
int transmissionState = RADIOLIB_ERR_NONE;
uint8_t radio_round_robin = 0;

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

void print_flash_around_address(uint32_t address)
{
  uint8_t readData[32];
  if (flash.readByteArray(address - 16, readData, 32))
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

uint32_t current_flash_address = 0x0;

void init_flash()
{
  USBSerial.println("[INFO] Searching for latest valid data.");

  while (true)
  {
    DataHeader header;
    if (flash.readByteArray(current_flash_address, (uint8_t *)&header, sizeof(DataHeader)))
    {
      if (header.packet_flag == 0xD3ADB33F)
      {
        // USBSerial.println("[INFO] Found valid data.");
        current_flash_address += header.data_size;
      }
      else
      {
        USBSerial.println("[INFO] Found invalid data.");
        break;
      }
    }
    else
    {
      USBSerial.println("[INFO] No data found.");
      break;
    }
  }

  USBSerial.print("[INFO] Current flash address: ");
  USBSerial.println(current_flash_address);

  // Get flash size
  uint32_t flash_size = flash.getCapacity();

  // Print percent used
  USBSerial.print("[INFO] Flash percent used: ");
  USBSerial.println((float)current_flash_address / flash_size * 100);

  // print first 16 bytes of flash around current_flash_address
  // uint8_t readData[32];
  // if (flash.readByteArray(current_flash_address - 16, &readData[0], 32))
  // {
  //   USBSerial.println("[INFO] Flash read success.");
  //   for (int i = 0; i < 32; i++)
  //   {
  //     USBSerial.print(readData[i], HEX);
  //     USBSerial.print(" ");
  //   }
  //   USBSerial.println();
  // }
  // else
  // {
  //   USBSerial.println("[ERROR] Flash read failed.");
  // }
}

unsigned long lastWrite = 0;
unsigned long totalWriteBytes = 0;
// Store 4096 bytes of data to flash
uint8_t flashData[4096];

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

  if (millis() - lastWrite > 5000)
  {
    USBSerial.print("[INFO] Current flash address: ");
    USBSerial.println(current_flash_address);
    lastWrite = millis();
  }

  if (flash.writeByteArray(current_flash_address, data, length, true))
  {
    // USBSerial.println("[INFO] Flash write success.");
    current_flash_address += length;
  }
  else
  {
    USBSerial.println("[INFO] Flash write failed, erasing partial.");

    // Check to see if the data crosses a sector boundary, if it does then we need to erase two sectors.
    // We will still need to copy the current data being erased in the lowest sector so we can rewrite it.
    uint32_t start_sector = current_flash_address / 4096;
    uint32_t end_sector = (current_flash_address + length) / 4096;
    uint32_t current_buffer_filled = current_flash_address % 4096;

    // Copy
    if (flash.readByteArray(start_sector * 4096, flashData, 4096))
    {
      // USBSerial.println("[INFO] Flash read success.");
    }
    else
    {
      USBSerial.println("[ERROR] Flash read failed.");
      led_panic();
    }

    // Erase the sector

    // Erase the sector
    if (!flash.eraseSector(start_sector * 4096))
    {
      USBSerial.println("[ERROR] Flash erase failed.");
      led_panic();
    }
    else
    {
      USBSerial.println("[INFO] Flash erase success.");
    }

    if (start_sector != end_sector)
    {
      // Insert data into until the end of the first sector
      for (uint32_t i = 0; i < 4096 - current_buffer_filled; i++)
      {
        flashData[current_buffer_filled + i] = data[i];
      }

      // Write the data from flashData back to the start of the sector
      if (flash.writeByteArray(start_sector * 4096, flashData, 4096, true))
      {
        USBSerial.println("[INFO] Flash write success.");
        current_flash_address += 4096 - current_buffer_filled;
      }
      else
      {
        USBSerial.println("[ERROR] Flash write failed.");
        led_panic();
      }

      if (!flash.eraseSector(end_sector * 4096))
      {
        USBSerial.println("[ERROR] Flash erase failed.");
        led_panic();
      }
      else
      {
        USBSerial.println("[INFO] Flash erase success.");
      }

      // Write the rest of the data from data back to the start of the second sector
      if (flash.writeByteArray(end_sector * 4096, data + 4096 - current_buffer_filled, length - (4096 - current_buffer_filled), true))
      {
        USBSerial.println("[INFO] Flash write success.");
        current_flash_address += length - (4096 - current_buffer_filled);
      }
      else
      {
        USBSerial.println("[ERROR] Flash write failed.");
        USBSerial.print("[INFO] Current flash address: ");
        USBSerial.println(current_flash_address);
        USBSerial.printf("[INFO] Start sector: %d\n", start_sector);
        USBSerial.printf("[INFO] End sector: %d\n", end_sector);
        USBSerial.printf("[INFO] Length: %d\n", length);
        USBSerial.printf("[INFO] Current buffer filled: %d\n", current_buffer_filled);

        led_panic();
      }
    }
    else
    {
      USBSerial.println("[INFO] Same sector.");

      // Insert data into flashData
      for (uint32_t i = 0; i < length; i++)
      {
        flashData[current_buffer_filled + i] = data[i];
      }

      // Write the data from flashData back to the start of the sector
      if (flash.writeByteArray(start_sector * 4096, flashData, current_buffer_filled + length, true))
      {
        USBSerial.println("[1INFO] Flash write success.");
        current_flash_address += current_buffer_filled + length;
      }
      else
      {
        USBSerial.println("[1ERROR] Flash write failed.");
        USBSerial.print("[INFO] Current flash address: ");
        USBSerial.println(current_flash_address);
        USBSerial.printf("[INFO] Start sector: %d\n", start_sector);
        USBSerial.printf("[INFO] End sector: %d\n", end_sector);
        USBSerial.printf("[INFO] Length: %d\n", length);
        USBSerial.printf("[INFO] Current buffer filled: %d\n", current_buffer_filled);
        led_panic();
      }
    }

    // USBSerial.print("[INFO] Current flash address: ");
    // USBSerial.println(current_flash_address);
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
  init_flash();

  //   //////////////////////////////////////////////////////////////////////////////////////////

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

    File root = SD.open("/");
    // Check for anything else named "rocket_data_raw_*.dat" and return
    // the highest number + 1
    int max = 0;
    while (true)
    {
      File entry = root.openNextFile();
      if (!entry)
      {
        break;
      }
      if (entry.isDirectory())
      {
        continue;
      }
      String name = entry.name();
      if (name.startsWith("rocket_data_raw_") && name.endsWith(".dat"))
      {
        int num = name.substring(16, name.length() - 4).toInt();
        if (num > max)
        {
          max = num;
        }
      }
      entry.close();
    }
    max++;
    root.close();

    // Print
    USBSerial.print("[INFO] Next file number: ");
    USBSerial.println(max + 1);

    if (current_flash_address != 0)
    {
      USBSerial.print("[INFO] Dumping data: ");
      USBSerial.println(current_flash_address);

      // Open file
      char filename[32];
      sprintf(filename, "/rocket_data_raw_%d.dat", max);
      File file = SD.open(filename, FILE_WRITE);

      // Write data
      for (uint32_t i = 0; i < current_flash_address; i += 4096)
      {
        uint8_t readData[4096];
        if (flash.readByteArray(i, readData, 4096))
        {
          file.write(readData, 4096);
        }
        else
        {
          USBSerial.println("[ERROR] Flash read failed.");
          led_panic();
        }
      }

      // Close file
      file.close();

      // Erase flash
      if (!flash.eraseSector(0x0))
      {
        USBSerial.println("[ERROR] Flash erase failed.");
        led_panic();
      }
      else
      {
        USBSerial.println("[INFO] Flash erase success.");
      }

      current_flash_address = 0;
      init_flash();
    }
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
  radio.setBandwidth(500.0);
  radio.setSpreadingFactor(10);
  radio.setCodingRate(5);
  radio.setPreambleLength(8);
  radio.setCRC(true);
  // radio.setPacketReceivedAction(setFlag);
  radio.setDio1Action(setFlag);
  radio.startReceive();
#endif

  gnss.setI2COutput(COM_TYPE_UBX);
  gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  gnss.setNavigationFrequency(5);

  // DeviceActivateData
  DeviceActivateData deviceActivateData;
  deviceActivateData.header.data_size = sizeof(DeviceActivateData);
  deviceActivateData.header.data_type = DATA_TYPE_DEVICE_ACTIVATE;

  write_flash((uint8_t *)&deviceActivateData, sizeof(DeviceActivateData));

  ledSetHigh();
}

unsigned long lastGPS = 0;
bool gpsLock = false;

unsigned long lastPressure = 0;

unsigned long lastIMU = 0;

unsigned long lastMag = 0;

unsigned long lastTime = 0; // Calculate loop time

unsigned long lastTransmit = 0;

GPSData gpsData;
PressureData pressureData;
IMUData imuData;
MagData magData;

float theta[3] = {0, 0, 0};

bool transmitFlag = false;

void loop()
{
  // // USBSerial.printf("Loop time: %d\n", millis() - lastTime);

  // runMagCalibrate(mag, 1711.0);

  unsigned long currentMillis = millis();

  //////////////////////////////////////////
  /// Radio Data
  ///

#ifdef TRANSMITTER
  if (currentMillis - lastTransmit > DATA_RADIO_RATE && transmissionState == RADIOLIB_ERR_NONE)
  {

    // GPS
    if (radio_round_robin == 0)
    {
      if (gpsLock)
      {
        transmissionState = radio.transmit((uint8_t *)&gpsData, sizeof(GPSData));
      }
      else
      {
        radio_round_robin++;
      }
    }

    // Pressure
    if (radio_round_robin == 1)
    {
      if (pressure_enabled)
      {
        transmissionState = radio.transmit((uint8_t *)&pressureData, sizeof(PressureData));
      }
      else
      {
        radio_round_robin++;
      }
    }

    // IMU
    if (radio_round_robin == 2)
    {
      if (imu_enabled)
      {
        transmissionState = radio.transmit((uint8_t *)&imuData, sizeof(IMUData));
      }
      else
      {
        radio_round_robin++;
      }
    }

    // Mag
    if (radio_round_robin == 3)
    {
      if (mag_enabled)
      {
        transmissionState = radio.transmit((uint8_t *)&magData, sizeof(MagData));
        radio_round_robin = 0;
      }
      else
      {
        radio_round_robin = 0;
      }
    }

    radio_round_robin++;
    lastTransmit = currentMillis;

    // wait 5 s
    // delay(1000);
    USBSerial.printf("Round robin: %d\n", radio_round_robin);
  }
#else
  if (receivedFlag)
  {
    receivedFlag = false;
    uint8_t data[64];
    uint len = sizeof(data);
    transmissionState = radio.receive(data, len);

    if (transmissionState == RADIOLIB_ERR_NONE)
    {
      USBSerial.print("[INFO] Received: ");
      for (int i = 0; i < len; i++)
      {
        USBSerial.print(data[i], HEX);
        USBSerial.print(" ");
      }
      USBSerial.println();
    }
    else
    {
      USBSerial.print("[ERROR] Receive error: ");
      USBSerial.println(transmissionState);
    }
  }
#endif

  return;

  //////////////////////////////////////////
  /// GPS Data
  ///

  if (!gpsLock && gps_enabled)
  {
    led_fade_update();
  }

  if (!gpsLock && currentMillis - lastGPS > DATA_GPS_RATE_NO_LOCK && gps_enabled)
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
    gpsData.header.data_size = sizeof(GPSData);
    gpsData.header.data_type = DATA_TYPE_GPS;
    gpsData.header.timestamp = millis();

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

  ////////////////////////////////////////
  // Pressure Data

  if (currentMillis - lastPressure > DATA_PRESSURE_RATE && pressure_enabled)
  {
    float pressure = ps.readPressureMillibars();
    float temperature = ps.readTemperatureC();

    pressureData.header.data_size = sizeof(PressureData);
    pressureData.header.data_type = DATA_TYPE_PRESSURE;
    pressureData.header.timestamp = millis();

    pressureData.pressure = pressure;
    pressureData.temperature = temperature;

    write_flash((uint8_t *)&pressureData, sizeof(PressureData));

    lastPressure = currentMillis;
  }

  // //////////////////////////////////////////
  // /// IMU Data
  // ///

  if (currentMillis - lastIMU > DATA_IMU_RATE && imu_enabled)
  {
    imu.read();

    imuData.header.data_size = sizeof(IMUData);
    imuData.header.data_type = DATA_TYPE_IMU;
    imuData.header.timestamp = millis();

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

  // //////////////////////////////////////////
  // /// Mag Data
  // ///

  if (currentMillis - lastMag > DATA_MAG_RATE && mag_enabled)
  {
    mag.read();

    magData.header.data_size = sizeof(MagData);
    magData.header.data_type = DATA_TYPE_MAG;
    magData.header.timestamp = millis();

    magData.mag_x = (float)mag.m.x / 1711.0;
    magData.mag_y = (float)mag.m.y / 1711.0;
    magData.mag_z = (float)mag.m.z / 1711.0;

    // USBSerial.printf("Mag: %f %f %f\n", magData.mag_x, magData.mag_y, magData.mag_z);

    write_flash((uint8_t *)&magData, sizeof(MagData));

    lastMag = currentMillis;
  }
}