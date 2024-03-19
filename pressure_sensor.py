import machine
from time import sleep

# Create I2C object
i2c = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16))

# Print out any addresses found
devices = i2c.scan()
print(devices)

if devices:
    for d in devices:
        print(hex(d))
        
exit(0)

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

base_height = 1004.2

while True:
    rslt = (float((int.from_bytes(reg_read(i2c, 0x5d, 0x2A), 'big')<<16) | (int.from_bytes(reg_read(i2c, 0x5d, 0x29), 'big')<<8) | (int.from_bytes(reg_read(i2c, 0x5d, 0x28), 'big')<<0))/4096)
    relative_height = 44330 * (1 - (rslt/base_height)**(1/5.255))
    print(f"{relative_height:0.2f}m", rslt)
    sleep(0.2)