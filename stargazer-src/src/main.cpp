#include <Arduino.h>

// SPDX-FileCopyrightText: 2023 Carter Nelson for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

#include <Wire.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n1I2C Scanner1");
  WIRE.begin();
  delay(3000);
  Serial.println("\nI2C Scanner2");
  WIRE.setClock(400000);
  // while (true)
  //   delay(10);
  delay(3000);
  Serial.println("\nI2C Scanner3");
}

void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  // Read from who am I register and print result from device 0x1f
  WIRE.beginTransmission(0x1f);
  WIRE.write(0x13);
  WIRE.endTransmission();
  WIRE.requestFrom(0x1f, 1);
  if (WIRE.available())
  {
    Serial.print("Who am I: ");
    Serial.println(WIRE.read(), HEX);
  }
  else
  {
    Serial.println("No response from device 0x1f");
  }

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
    else
    {
      Serial.print("No device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for next scan
}
