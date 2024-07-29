import struct
import csv

# Define the struct formats
data_header_format = (
    "<IIII"  # Read 4 unsigned ints (packet_flag, data_size, data_type, timestamp)
)
gps_data_format = "<HBBBBBBiiIIIBBBB"
pressure_data_format = "<ff"
imu_data_format = "<hhhhhh"
mag_data_format = "<fff"

# Define the data type identifiers
DATA_TYPE_GPS = 0x01
DATA_TYPE_PRESSURE = 0x02
DATA_TYPE_IMU = 0x03
DATA_TYPE_MAG = 0x04
DATA_TYPE_DEVICE_ACTIVATE = 0x05


# Function to read and parse the data from the dat file
def parse_dat_file(file_path):
    parsed_data = []
    with open(file_path, "rb") as file:
        while True:
            # Read the header (16 bytes)
            header_data = file.read(16)
            if len(header_data) < 16:
                break  # End of file

            packet_flag, data_size, data_type, timestamp = struct.unpack(
                data_header_format, header_data
            )

            # Verify the packet flag
            if packet_flag != 0xD3ADB33F:
                print("Invalid packet flag")
                break

            # Read and parse data based on the data_type
            if data_type == DATA_TYPE_GPS:
                gps_data = file.read(struct.calcsize(gps_data_format))
                if len(gps_data) < struct.calcsize(gps_data_format):
                    break
                parsed_data.append(
                    ["GPS", timestamp] + list(struct.unpack(gps_data_format, gps_data))
                )

                print(
                    list(struct.unpack(gps_data_format, gps_data))[7] / 10000000,
                    list(struct.unpack(gps_data_format, gps_data))[8] / 10000000,
                )

            elif data_type == DATA_TYPE_PRESSURE:
                pressure_data = file.read(struct.calcsize(pressure_data_format))
                if len(pressure_data) < struct.calcsize(pressure_data_format):
                    break
                parsed_data.append(
                    ["Pressure", timestamp]
                    + list(struct.unpack(pressure_data_format, pressure_data))
                )

            elif data_type == DATA_TYPE_IMU:
                imu_data = file.read(struct.calcsize(imu_data_format))
                if len(imu_data) < struct.calcsize(imu_data_format):
                    break
                parsed_data.append(
                    ["IMU", timestamp] + list(struct.unpack(imu_data_format, imu_data))
                )

            elif data_type == DATA_TYPE_MAG:
                mag_data = file.read(struct.calcsize(mag_data_format))
                if len(mag_data) < struct.calcsize(mag_data_format):
                    break
                parsed_data.append(
                    ["Mag", timestamp] + list(struct.unpack(mag_data_format, mag_data))
                )

            elif data_type == DATA_TYPE_DEVICE_ACTIVATE:
                parsed_data.append(["DeviceActivate", timestamp])
                # No additional data to read

            else:
                print(f"Unknown data type: {data_type}")
                break

    return parsed_data


def export_to_csv(parsed_data, output_csv_path):
    # Define headers for each data type
    headers = [
        "Data Type",
        "Timestamp",
        "Year",
        "Month",
        "Day",
        "Hour",
        "Minute",
        "Second",
        "Latitude",
        "Longitude",
        "Altitude",
        "Speed",
        "Heading",
        "Satellites",
        "Fix",
        "Pressure",
        "Temperature",
        "Acc_X",
        "Acc_Y",
        "Acc_Z",
        "Gyro_X",
        "Gyro_Y",
        "Gyro_Z",
        "Mag_X",
        "Mag_Y",
        "Mag_Z",
    ]

    with open(output_csv_path, "w", newline="") as csvfile:
        csvwriter = csv.writer(csvfile)

        # Write headers
        csvwriter.writerow(headers)

        # Write the data rows
        for row in parsed_data:
            csvwriter.writerow(row)


# Path to the dat file
dat_file_path = "rocket_data_raw_1.dat"

# Path to the output CSV file
output_csv_path = "parsed_data.csv"

# Parse the dat file
parsed_data = parse_dat_file(dat_file_path)

# Export the parsed data to a CSV file
export_to_csv(parsed_data, output_csv_path)
