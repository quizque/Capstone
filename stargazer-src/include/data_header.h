#define DATA_TYPE_GPS 0x01
#define DATA_TYPE_PRESSURE 0x02
#define DATA_TYPE_IMU 0x03
#define DATA_TYPE_MAG 0x04
#define DATA_TYPE_DEVICE_ACTIVATE 0x05
#define DATA_TYPE_TRANSMIT 0x06

struct DataHeader
{
    unsigned int packet_flag = 0xD3ADB33F;
    unsigned int data_size;
    unsigned int data_type;
    unsigned int timestamp;
};

struct GPSData
{
    DataHeader header;

    uint16_t year;
    uint8_t month;
    uint8_t day;

    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    uint32_t latitude;
    uint32_t longitude;
    uint32_t altitude;

    uint32_t speed;
    uint32_t heading;

    uint8_t satellites;
    uint8_t fix;
};

struct PressureData
{
    DataHeader header;

    float pressure;
    float temperature;
};

struct IMUData
{
    DataHeader header;

    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

struct MagData
{
    DataHeader header;

    float mag_x;
    float mag_y;
    float mag_z;
};

struct DeviceActivateData
{
    DataHeader header;
};

struct TransmitData
{
    DataHeader header;

    uint32_t lat;
    uint32_t lon;
    float height;
};