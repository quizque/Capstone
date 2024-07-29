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
    spreading_factor=12,
)

led = Pin(25, Pin.OUT)

# Receive handler
def handler(x):
    # Echo message
    lora.send(x)
    print(x)
    print(f"Ours: RSSI: {lora.get_rssi()} SNR: {lora.get_snr()}")
    led.toggle()
    # Put module back in recv mode
    lora.recv()

# Set handler
lora.on_recv(handler)
# Put module in recv mode
lora.recv()
print("waiting")
# No need for main loop, code is asynchronous 