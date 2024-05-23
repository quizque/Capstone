#include <Arduino.h>
#include <SPI.h>

#include <SD.h>

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(7, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(10, HIGH);
  SPI.begin();

  delay(5000);

  Serial.print("\nInitializing SD card...");

  if (!SD.begin(10))
  {
    Serial.println("SD Mount Failed");
  }

  File myFile;
  myFile = SD.open("/test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile)
  {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:

  myFile = SD.open("/test.txt");

  if (myFile)
  {

    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:

    while (myFile.available())
    {

      Serial.write(myFile.read());
    }

    // close the file:

    myFile.close();
  }
  else
  {

    // if the file didn't open, print an error:

    Serial.println("error opening test.txt");
  }
}

void loop()
{
  // nothing happens after setup
}