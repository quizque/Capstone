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

#define TRANSMITTER 1

#define USE_LORA
// #define USING_SX1262

#define FLASH_MARKER_PRIMARY 0x8
#define FLASH_MARKER_SECONDARY 0xC

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

void led_panic()
{
  while (1)
  {
    digitalWrite(LED_STATUS, LOW);
    delay(100);
    digitalWrite(LED_STATUS, HIGH);
    delay(100);
  }
}

uint32_t current_flash_address = 0x0;
const char *flash_marker = "STARGAZE";

void init_flash()
{
  uint8_t flashData[9];
  flash.readByteArray(0x0, &flashData[0], 8);

  if (memcmp(flashData, flash_marker, 8) != 1)
  {
    USBSerial.println("[WARNING] Flash not initialized, setting up...");
    flash.eraseBlock64K(0x0);
    flash.writeByteArray(0x0, (uint8_t *)flash_marker, 8, true);

    flash.writeULong(FLASH_MARKER_PRIMARY, 16);
    flash.writeULong(FLASH_MARKER_SECONDARY, 16);
    USBSerial.println("[INFO] Flash initialized.");
  }
  else
  {
    USBSerial.println("[INFO] Flash already initialized.");
  }

  USBSerial.println("[INFO] Getting current flash address.");
  uint32_t primary_marker = 0x0;
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

  pinMode(LED_STATUS, OUTPUT);
  analogWrite(LED_STATUS, 125);
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
  }

  // Check gyro/acc
  if (!imu.init())
  {
    USBSerial.println("[WARNING] Gyro/acc not found, continuing.");
  }
  else
  {
    USBSerial.println("[INFO] IMU found.");
  }

  // Check Magnetometer
  if (!mag.init())
  {
    USBSerial.println("[WARNING] Magnetometer not found, continuing.");
  }
  else
  {
    USBSerial.println("[INFO] Magnetometer found.");
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
  }

  USBSerial.println("[INFO] Initialization complete.");

  ///
  /// Device setup complete
  ///

#ifdef USE_LORA
  radio.setFrequency(915.0);
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

  USBSerial.print(flash.readByte(0x12), HEX);
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

long lastTime = 0;

void loop()
{
  // gnss.checkUblox();
  // if (millis() - lastTime > 1000)
  // {
  //   lastTime = millis(); // Update the timer

  //   long latitude = gnss.getLatitude();
  //   USBSerial.print(F("Lat: "));
  //   USBSerial.print(latitude);

  //   long longitude = gnss.getLongitude();
  //   USBSerial.print(F(" Long: "));
  //   USBSerial.print(longitude);
  //   USBSerial.print(F(" (degrees * 10^-7)"));

  //   long altitude = gnss.getAltitude();
  //   USBSerial.print(F(" Alt: "));
  //   USBSerial.print(altitude);
  //   USBSerial.print(F(" (mm)"));

  //   byte SIV = gnss.getSIV();
  //   USBSerial.print(F(" SIV: "));
  //   USBSerial.print(SIV);

  //   int satellites = gnss.getYear();
  //   USBSerial.print(F(" Satellites: "));
  //   USBSerial.print(satellites);

  //   USBSerial.println();
  // }
}