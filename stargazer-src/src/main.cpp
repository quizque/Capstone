

#include <Arduino.h>

#define Serial USBSerial
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_SensorLab.h>
#include <Adafruit_Sensor_Calibration.h>

Adafruit_LIS3MDL lis3mdl;

// Hard-iron calibration settings
const float hard_iron[3] = {
    -32.71, -7.94, -10.54};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
    {0.992, 0.007, 0.011},
    {0.007, 1.011, -0.024},
    {0.011, -0.024, 0.998}};

const float mag_decl = -1.233;

void setup()
{
  // Open serial communications and wait for port to open:
  // Serial0.begin(9600);
  // Serial.begin(9600, SERIAL_8N1, 20, 21);
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

  // USBSerial.print("\nAAA");

  Wire.begin();

  Serial.println(F("Sensor Lab - IMU Calibration!"));

  Serial.println("LIS3MDL compass test");

  // Initialize magnetometer
  if (!lis3mdl.begin_I2C(0x1E))
  {
    Serial.println("ERROR: Could not find magnetometer");
    while (1)
    {
      delay(1000);
    }
  }
}

void loop()
{
  // if (USBSerial.available())
  // { // If anything comes in Serial (USB),
  //   // USBSerial.print("USBSerial available");
  //   Serial.write(USBSerial.read()); // read it and send it out Serial1 (pins 4 & 5)
  // }

  // if (Serial.available())
  // {                                 // If anything comes in Serial1 (pins 4 & 5)
  //   USBSerial.write(Serial.read()); // read it and send it out Serial (USB)
  // }
  // float pressure = ps.readPressureMillibars();
  // float altitude = ps.pressureToAltitudeMeters(pressure);
  // float temperature = ps.readTemperatureC();

  // USBSerial.print("p: ");
  // USBSerial.print(pressure);
  // USBSerial.print(" mbar\ta: ");
  // USBSerial.print(altitude);
  // USBSerial.print(" m\tt: ");
  // USBSerial.print(temperature);

  static float hi_cal[3];
  static float heading = 0;

  // Get new sensor event with readings in uTesla
  sensors_event_t event;
  lis3mdl.getEvent(&event);

  // Put raw magnetometer readings into an array
  float mag_data[] = {event.magnetic.x,
                      event.magnetic.y,
                      event.magnetic.z};

  // Apply hard-iron offsets
  for (uint8_t i = 0; i < 3; i++)
  {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }

  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++)
  {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
  }

  // Calculate angle for heading, assuming board is parallel to
  // the ground and  Y points toward heading.
  heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI +180;

  // Apply magnetic declination to convert magnetic heading
  // to geographic heading
  // heading = heading + mag_decl;

  // Convert heading to 0..360 degrees
  // if (heading < 0)
  // {
  //   heading = 360;
  // }

  // Print calibrated results
  Serial.print("[");
  Serial.print(mag_data[0], 1);
  Serial.print("\t");
  Serial.print(mag_data[1], 1);
  Serial.print("\t");
  Serial.print(mag_data[2], 1);
  Serial.print("] Heading: ");
  Serial.println(heading, 2);
}