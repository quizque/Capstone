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

/////////////////////////////////////////////
// Configuration

// Enable/disable transmitter mode and select
// the type of transmitter

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

SFE_UBLOX_GNSS gnss;

// Enabled devices
bool gps_enabled = false;
bool pressure_enabled = false;
bool imu_enabled = false;
bool mag_enabled = false;
bool high_imu = false;

// Is high data rate enabled
bool override_enabled = false;

// Transmission state
volatile bool operationDone = false;
int transmissionState = RADIOLIB_ERR_NONE;

// Transmission round robin tracker
uint8_t radio_round_robin = 0;

/////////////////////////////////////////////
// LED Controls

short unsigned int led_current_pwm = 0;
unsigned long int led_current_time = 0;
bool led_up = true;

// LED fade update
// Call continuously to make the LED fade in and out
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
  // ESP-IDF specific function to write to PWM
  ledcWrite(0, led_current_pwm);
}

// Set the LED to high (100% duty cycle)
void ledSetHigh()
{
  ledcWrite(0, 255);
}

// Set the LED to low (0% duty cycle)
void ledSetLow()
{
  ledcWrite(0, 0);
}

// LED panic function
// Blinks the LED on and off and halts the program.
// Only use in critical errors.
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

/////////////////////////////////////////////
// Flash memory functions

// Current flash address
uint32_t current_flash_address = 0x0;

// Measure the write rate of the flash memory
unsigned long lastWrite = 0;
unsigned long totalWriteBytes = 0;

// Store 4096 bytes of data to flash
uint8_t flashData[4096];

// Used to measure the flash percent filled
uint32_t time_since_last_flash_noti = 0;
uint32_t flash_size = 1;

// Print the flash memory around a specific address
// Used for debugging
void print_flash_around_address(uint32_t address)
{
  // Buffer to store the data
  uint8_t readData[32];

  // Read 32 bytes of data from the flash at the address
  if (flash.readByteArray(address, readData, 32))
  {
    // Print the data
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

// Initialize the flash memory
// This function will search for the latest valid data
// and set the current_flash_address to the next available address
// to write data to.
// This has no alignment checks, crc checks, safety checks, etc. so
// it will only work if the data is written correctly.
void init_flash()
{
  USBSerial.println("[INFO] Searching for latest valid data.");

  // This has no safety checks, it will (maybe?) loop forever if the flash is full
  while (true)
  {
    // Header to store the data to be checked
    DataHeader header;

    // Read the data at the current address
    if (flash.readByteArray(current_flash_address, (uint8_t *)&header, sizeof(DataHeader)))
    {
      // Check if the data is valid
      if (header.packet_flag == 0xD3ADB33F)
      {
        // Packet is valid, move the current_flash_address
        // to the next available address to write data to
        current_flash_address += header.data_size;
      }
      else
      {
        // Packet is invalid, break the loop
        // This isn't good and will probably cause data corruption
        USBSerial.println("[INFO] Found invalid data.");
        break;
      }
    }
    else
    {
      // The library will return false if the data is out of bounds, but from one test
      // it didn't, so hopefully this never happens
      USBSerial.println("[INFO] No data found.");
      break;
    }
  }

  // Print the current flash address
  USBSerial.print("[INFO] Current flash address: ");
  USBSerial.println(current_flash_address);
}

// Write data to the flash memory
// Performs automatic sector erases, alignment, and boundary checks
void write_flash(uint8_t *data, uint32_t length)
{
#ifdef MEASURE_STORE_RATE

  // Measure the write rate
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

  // Display percent filled every 5 seconds
  if (millis() - time_since_last_flash_noti > 5000)
  {
    USBSerial.print("[INFO] Flash percent filled: ");
    USBSerial.println((float)current_flash_address / flash_size * 100);
    time_since_last_flash_noti = millis();
  }

  // Try to write the data to the flash memory
  if (flash.writeByteArray(current_flash_address, data, length, true))
  {
    current_flash_address += length;
  }
  // If the write fails, erase the sector and try again
  else
  {
    USBSerial.println("[INFO] Flash write failed, erasing partial.");

    // Check to see if the data crosses a sector boundary, if it does then we need to erase two sectors.
    // We will still need to copy the current data being erased in the lowest sector so we can rewrite it.
    uint32_t start_sector = current_flash_address / 4096;
    uint32_t end_sector = (current_flash_address + length) / 4096;
    uint32_t current_buffer_filled = current_flash_address % 4096;

    // Copy the data from the current sector to flashData
    if (!flash.readByteArray(start_sector * 4096, flashData, 4096))
    {
      // If we end up here, the flash is probably dead
      USBSerial.println("[ERROR] Flash read failed.");
      led_panic();
    }

    // Erase the sector
    if (!flash.eraseSector(start_sector * 4096))
    {
      USBSerial.println("[ERROR] Flash erase failed.");
      led_panic();
    }

    // Check if the data crosses a sector boundary
    // This will fail if it crosses more then two sectors
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
        // Move the current flash address to the start of the next sector
        current_flash_address += 4096 - current_buffer_filled;
      }
      else
      {
        USBSerial.println("[ERROR] Flash write failed.");
        led_panic();
      }

      // Erase the next sector
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
        // Debugging
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
    // Same sector
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
        // Move the current flash address to the end of the data
        current_flash_address += current_buffer_filled + length;
      }
      else
      {
        // Debugging
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

  // Check to make sure the flash is the right one
  // Any should work, but its only tested with the one in the schematic
  uint32_t JEDEC = flash.getJEDECID();
  if (0xef4018 != JEDEC)
  {
    USBSerial.println("[ERROR] Flash not found, halting.");
    led_panic();
  }
  USBSerial.println("[INFO] Flash found.");
  init_flash();

  ////////////////////////////////////////////////////////////////////////////////////////////

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

    // Check to see what the next file number should be
    // so that we don't overwrite any previous data

    File root = SD.open("/");
    // Check for anything else named "rocket_data_raw_*.dat" and return
    // the highest number + 1
    int max = 0;
    while (true)
    {
      File entry = root.openNextFile();

      // No more files
      if (!entry)
      {
        break;
      }

      // Skip directories
      if (entry.isDirectory())
      {
        continue;
      }

      // Check the name
      String name = entry.name();
      if (name.startsWith("rocket_data_raw_") && name.endsWith(".dat"))
      {
        // Extract the number from the name
        int num = name.substring(16, name.length() - 4).toInt();
        // Check if it is the highest number
        if (num > max)
        {
          max = num;
        }
      }

      entry.close();
    }

    // Increment the number
    max++;

    root.close();

    // Print
    USBSerial.print("[INFO] Next file number: ");
    USBSerial.println(max + 1);

    // Check if there is any data in the flash memory
    // If there is, dump it to the SD card
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
        // Read data from flash (we can only read 4096 bytes at a time)
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
      // Only erase the first sector, this will make it look like the flash is empty
      if (!flash.eraseSector(0x0))
      {
        USBSerial.println("[ERROR] Flash erase failed.");
        led_panic();
      }
      else
      {
        USBSerial.println("[INFO] Flash erase success.");
      }

      // Reset the current flash address and reinitialize the flash
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
    // Set the accelerometer to 8g and the gyro to 1000dps
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
    // Set the magnetometer to 4 gauss
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

  // Configure radio
#ifdef USE_LORA
  radio.setFrequency(919.0);
  radio.setBandwidth(500.0);
  radio.setSpreadingFactor(10);
  radio.setCodingRate(5);
  radio.setPreambleLength(8);
  radio.setOutputPower(RADIOLIB_SX1278_MAX_POWER);
  radio.setCRC(true);
#endif

  // Configure GPS
  gnss.setI2COutput(COM_TYPE_UBX);
  gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  gnss.setNavigationFrequency(5);

  // Save a special data type to the flash memory to
  // indicate that the device has been activated
  DeviceActivateData deviceActivateData;
  deviceActivateData.header.data_size = sizeof(DeviceActivateData);
  deviceActivateData.header.data_type = DATA_TYPE_DEVICE_ACTIVATE;

  write_flash((uint8_t *)&deviceActivateData, sizeof(DeviceActivateData));

  // Get the flash size
  flash_size = flash.getCapacity();
}

// Do we have GPS Lock
bool gpsLock = false;

// Poor mans round robin tracker
unsigned long lastGPS = 0;
unsigned long lastPressure = 0;
unsigned long lastIMU = 0;
unsigned long lastMag = 0;

// Used to calculate how long a transmission and loop if
unsigned long lastTransmit = 0;
unsigned long lastRadio = 0;

// Used to keep track of when to save/transmit data in override mode
uint32_t lastOverdrive = 0;

// Holds the current data packets so we
// can save them to the flash
GPSData gpsData;
PressureData pressureData;
IMUData imuData;
MagData magData;

// Used to blink the status LED
// Only  used to show when we got a packet as a receiver
bool led_toggle = false;

void loop()
{
  // When is this loop occurring
  unsigned long currentMillis = millis();

#ifdef TRANSMITTER

  // Currently configured to only transmit GPS and height

  //////////////////////////////////////////////////////////////
  // Transmitter code

  // If the time since we last transmitted is longer then we configured,
  // send a new packet
  if (currentMillis - lastTransmit > DATA_RADIO_RATE && transmissionState == RADIOLIB_ERR_NONE)
  {
    // Define packet
    TransmitData td;

    // Insert data
    td.header.data_size = sizeof(TransmitData);
    td.header.data_type = DATA_TYPE_TRANSMIT;
    td.header.timestamp = millis();

    // If we have lock, transmit it
    if (gpsLock)
    {
      td.lat = gpsData.latitude;
      td.lon = gpsData.longitude;
    }
    // Otherwise, just send -1
    else
    {
      td.lat = 1;
      td.lon = 1;
    }

    // Read and convert the pressure sensor value
    td.height = ps.pressureToAltitudeMeters(ps.readPressureMillibars());

    // Transmit the signal and save the state
    transmissionState = radio.transmit((uint8_t *)&td, sizeof(TransmitData));

    // Save the last time we transmitted so we know when to do it again
    lastTransmit = millis();
  }

#else

  //////////////////////////////////////////////////////////////
  // Transmitter code

  // Buffer for received data
  uint8_t data[128];
  transmissionState = radio.receive(data, 128);

  if (transmissionState == RADIOLIB_ERR_NONE)
  {
    // Cast the data to a struct
    TransmitData *td = (TransmitData *)data;

    // Check to make sure the header is correct
    // (i.e. this is our data, not someone elses)
    if (td->header.packet_flag != 0xD3ADB33F)
      return;

    // Debug
    USBSerial.print("[INFO] Received: ");
    USBSerial.printf("%d %d %d %f", td->header.timestamp, td->lon, td->lat, td->height);

    // Write result to flash
    write_flash((uint8_t *)&td, sizeof(TransmitData));

    // Toggle led to show we received data
    led_toggle = !led_toggle;

    // LED toggle code
    if (led_toggle)
    {
      ledSetHigh();
    }
    else
    {
      ledSetLow();
    }
  }
  else
  {
    // Radio error, usually received packet wrong
    USBSerial.println("e");
  }

  return;
#endif

  //////////////////////////////////////////////////////////////
  // High seed data saving

  //////////////////////////////////////////////////////////////
  // GPS packet

  // If we don't have lock, fade the LED
  if (!gpsLock)
  {
    led_fade_update();
  }

  // If we have NO lock, gps is enabled, and its time to save (overdrive disabled here)
  // We don't save here since we don't have any data, its just named that for consistantancy
  if ((!gpsLock && currentMillis - lastGPS > DATA_GPS_RATE_NO_LOCK && gps_enabled) ||
      (!gpsLock && currentMillis - lastGPS > DATA_GPS_RATE_NO_LOCK * 3 && gps_enabled))
  {
    // Check if we have lock
    if (gnss.getFixType() != 0)
    {
      USBSerial.println("LOCK OBTAINED");
      ledSetHigh();
      gpsLock = true;
    }
    // No lock :(
    else
    {
      USBSerial.println("NO LOCK OBTAINED");

      gpsLock = false;
    }

    lastGPS = currentMillis;
  }

  // If we have lock, gps is enabled, and its time to save
  if ((gpsLock && currentMillis - lastGPS > DATA_GPS_RATE_LOCK && gps_enabled) ||
      (gpsLock && currentMillis - lastGPS > DATA_GPS_RATE_LOCK * 3 && gps_enabled))
  {
    // Fill the GPS data struct
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

    // If we lose GPS lock, switch the LED to fade
    if (gpsData.fix == 0)
    {
      USBSerial.println("NO LOCK OBTAINED");
      gpsLock = false;
    }
    else
    {
      // Debug
      USBSerial.println("LOCK OBTAINED");
      USBSerial.printf("Lat: %d Long: %d Alt: %d Speed: %d Sats: %d Fix: %d\n", gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.speed, gpsData.satellites, gpsData.fix);

      // Save the data to the flash
      write_flash((uint8_t *)&gpsData, sizeof(GPSData));
      lastGPS = currentMillis;
    }
  }

  //////////////////////////////////////////
  // Pressure Data

  // If its time to save pressure data, its enabled
  if ((currentMillis - lastPressure > DATA_PRESSURE_RATE && pressure_enabled && lastOverdrive > currentMillis) ||
      (currentMillis - lastPressure > DATA_PRESSURE_RATE * 3 && pressure_enabled && lastOverdrive < currentMillis))
  {
    // Read the pressure and temperature
    float pressure = ps.readPressureMillibars();
    float temperature = ps.readTemperatureC();

    // Assemble the data
    pressureData.header.data_size = sizeof(PressureData);
    pressureData.header.data_type = DATA_TYPE_PRESSURE;
    pressureData.header.timestamp = millis();

    pressureData.pressure = pressure;
    pressureData.temperature = temperature;

    // Write the data to the flash
    write_flash((uint8_t *)&pressureData, sizeof(PressureData));

    // Save the last time we saved the data
    lastPressure = currentMillis;
  }

  ////////////////////////////////////////////
  // IMU Data

  // Overdrive activation code
  if (imu_enabled)
  {
    imu.read();

    // Convert the accelerometer data to g
    imu.a.x = imu.a.x * 0.488 / 1000.0;
    imu.a.y = imu.a.y * 0.488 / 1000.0;
    imu.a.z = imu.a.z * 0.488 / 1000.0;

    // Calculate the
    float acc = sqrt(imu.a.x * imu.a.x + imu.a.y * imu.a.y + imu.a.z * imu.a.z);

    // If we are over the specified limit, activate overdrive
    if (acc > ACC_OVERDRIVE_WRITE)
    {
      USBSerial.println("OVERDRIVE");
      lastOverdrive = currentMillis + 20000;
    }
  }

  // If its time to save the IMU data and its enabled
  if ((currentMillis - lastIMU > DATA_IMU_RATE && imu_enabled && lastOverdrive > currentMillis) ||
      (currentMillis - lastIMU > DATA_IMU_RATE * 3 && imu_enabled && lastOverdrive < currentMillis))
  {
    // Read the data from the IMU
    imu.read();

    // Assemble the data
    imuData.header.data_size = sizeof(IMUData);
    imuData.header.data_type = DATA_TYPE_IMU;
    imuData.header.timestamp = millis();

    imuData.acc_x = imu.a.x * 0.488;
    imuData.acc_y = imu.a.y * 0.488;
    imuData.acc_z = imu.a.z * 0.488;

    imuData.gyro_x = imu.g.x * 35.0;
    imuData.gyro_y = imu.g.y * 35.0;
    imuData.gyro_z = imu.g.z * 35.0;

    // Write data to flash
    write_flash((uint8_t *)&imuData, sizeof(IMUData));

    lastIMU = currentMillis;
  }

  ////////////////////////////////////////////
  // Mag Data

  // If its time to save the IMU data and its enabled
  if ((currentMillis - lastMag > DATA_MAG_RATE && mag_enabled && lastOverdrive > currentMillis) ||
      (currentMillis - lastMag > DATA_MAG_RATE * 3 && mag_enabled && lastOverdrive < currentMillis))
  {
    // Read mag data
    mag.read();

    // Assemble the packet
    magData.header.data_size = sizeof(MagData);
    magData.header.data_type = DATA_TYPE_MAG;
    magData.header.timestamp = millis();

    magData.mag_x = (float)mag.m.x / 1711.0;
    magData.mag_y = (float)mag.m.y / 1711.0;
    magData.mag_z = (float)mag.m.z / 1711.0;

    // Write to flash
    write_flash((uint8_t *)&magData, sizeof(MagData));

    lastMag = currentMillis;
  }
}