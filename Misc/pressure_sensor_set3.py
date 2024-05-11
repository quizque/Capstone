import machine
from time import sleep, ticks_us
#from bmp280 import *

# Create I2C object
i2c = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16))
# Print out any addresses found
devices = i2c.scan()
print(devices)

exit(1)
bmp = BMP280(i2c, 0x77)


bmp.temp_os = BMP280_TEMP_OS_2
bmp.press_os = BMP280_PRES_OS_16
bmp.iir = BMP280_IIR_FILTER_OFF


if devices:
    for d in devices:
        print(hex(d))
        
(0)

def reg_write(i2c, addr, reg, data):
    """
    Write bytes to the specified register.
    """
    
    # Construct message
    msg = bytearray()
    msg.append(data)
    
    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)
    
def reg_read(i2c, addr, reg, nbytes=1):
    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """
    
    # Check to make sure caller is asking for 1 or more bytes
    if nbytes < 1:
        return bytearray()
    
    # Request data from specified register(s) over I2C
    data = i2c.readfrom_mem(addr, reg, nbytes)
    
    return data

def get_pressure():
    return (float((int.from_bytes(reg_read(i2c, 0x5d, 0x2A), 'big')<<16) | (int.from_bytes(reg_read(i2c, 0x5d, 0x29), 'big')<<8) | (int.from_bytes(reg_read(i2c, 0x5d, 0x28), 'big')<<0))/4096)

def get_height(press):
    rslt = press
    relative_height = 145366.45 * (1 - (rslt/1013.25)**(0.190284)) * 0.3048
    return relative_height

#print(reg_write(i2c, 0x76, 0x??, 0b))

#print(reg_write(i2c, 0x5d, 0x20, 0b11000100))


#base = get_height(get_pressure())


print("time,BMP280")

c_time = ticks_us()

bmp.normal_measure()

while True:
    print(f"{(ticks_us()-c_time)/(10**6)},{bmp.pressure/100}")
    sleep(1/25)