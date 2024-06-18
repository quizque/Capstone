#pragma once

void runMagCalibrate(LIS3MDL &mag, float scaleValue)
{
    static float mag_max[3] = {-4000, -4000, -4000};
    static float mag_min[3] = {4000, 4000, 4000};
    static float mag_offsets[3] = {0, 0, 0};
    static float mag_scale[3] = {0, 0, 0};

    float mag_raw[3] = {0, 0, 0};

    uint32_t notify_timer = 0;
    USBSerial.println("Calibrating magnetometer...");

    while (true)
    {
        mag.read();

        mag_raw[0] = (float)mag.m.x / scaleValue;
        mag_raw[1] = (float)mag.m.y / scaleValue;
        mag_raw[2] = (float)mag.m.z / scaleValue;

        for (int i = 0; i < 3; i++)
        {
            if (mag_raw[i] > mag_max[i])
                mag_max[i] = mag_raw[i];
            if (mag_raw[i] < mag_min[i])
                mag_min[i] = mag_raw[i];
        }

        for (int i = 0; i < 3; i++)
        {
            mag_offsets[i] = (mag_max[i] + mag_min[i]) / 2;
            mag_scale[i] = (mag_max[i] - mag_min[i]) / 2;
        }

        if (millis() - notify_timer > 1000)
        {
            notify_timer = millis();

            // Print the offset and scale
            USBSerial.print("Offsets: ");
            USBSerial.print(mag_offsets[0]);
            USBSerial.print(", ");
            USBSerial.print(mag_offsets[1]);
            USBSerial.print(", ");
            USBSerial.println(mag_offsets[2]);

            USBSerial.print("Scale: ");
            USBSerial.print(mag_scale[0]);
            USBSerial.print(", ");
            USBSerial.print(mag_scale[1]);
            USBSerial.print(", ");
            USBSerial.println(mag_scale[2]);

            // Print the process and raw values with printf
            USBSerial.print("Process: ");
            USBSerial.print((mag_raw[0] - mag_offsets[0]) / mag_scale[0]);
            USBSerial.print(", ");
            USBSerial.print((mag_raw[1] - mag_offsets[1]) / mag_scale[1]);
            USBSerial.print(", ");
            USBSerial.println((mag_raw[2] - mag_offsets[2]) / mag_scale[2]);

            USBSerial.print("Raw: ");
            USBSerial.print(mag_raw[0]);
            USBSerial.print(", ");
            USBSerial.print(mag_raw[1]);
            USBSerial.print(", ");
            USBSerial.println(mag_raw[2]);

            USBSerial.println();
        }
    }
}