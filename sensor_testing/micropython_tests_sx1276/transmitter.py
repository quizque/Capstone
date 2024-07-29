# Send "Hello world!" using the default LoRa parameters every second.
#
# The pin configuration used here is for the first LoRa module of these boards:
# https://makerfabs.com/esp32-lora-gateway.html

from lora import LoRa
from machine import Pin, SPI
from time import sleep

# SPI pins
SCK  = 2
MOSI = 3
MISO = 4
# Chip select
CS   = 16
# Receive IRQ
RX   = 17

# Setup SPI
spi = SPI(
    0,
    baudrate=5000000,
    sck=Pin(SCK, Pin.OUT, Pin.PULL_DOWN),
    mosi=Pin(MOSI, Pin.OUT, Pin.PULL_UP),
    miso=Pin(MISO, Pin.IN, Pin.PULL_UP),
)
spi.init()

# Setup LoRa
lora = LoRa(
    spi,
    cs=Pin(CS, Pin.OUT),
    rx=Pin(RX, Pin.IN),
    frequency=915.0,
    bandwidth=500000,
    spreading_factor=10,
    coding_rate=4,
)

print(lora.get_rssi())
print(lora.get_snr())
led = Pin(25, Pin.OUT)

#print(lora.set_frequency(kw.get('frequency', 907.0)))
#print(lora.set_bandwidth(kw.get('bandwidth', 500000)))

while True:
    lora.send(f"RSSI: {lora.get_rssi()} SNR: {lora.get_snr()}")
    print('yay')
    led.toggle()
    sleep(1)