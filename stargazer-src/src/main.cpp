#include <Arduino.h>
#include <Wire.h>

void setup()
{
  // Open serial communications and wait for port to open:
  // Serial0.begin(9600);
  Serial.begin(9600, SERIAL_8N1, 20, 21);
  USBSerial.begin(115200);
  pinMode(7, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(7, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(10, HIGH);
  // SPI.begin();
  delay(5000);

  USBSerial.print("\nReading serial 1");

  USBSerial.print("\nAAA");
}

void loop()
{
  if (USBSerial.available())
  { // If anything comes in Serial (USB),
    USBSerial.print("USBSerial available");
    Serial.write(USBSerial.read()); // read it and send it out Serial1 (pins 4 & 5)
  }

  if (Serial.available())
  {                                 // If anything comes in Serial1 (pins 4 & 5)
    USBSerial.write(Serial.read()); // read it and send it out Serial (USB)
  }
}