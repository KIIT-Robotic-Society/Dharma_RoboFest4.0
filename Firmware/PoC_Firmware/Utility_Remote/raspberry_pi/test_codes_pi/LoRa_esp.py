import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500000
spi.mode = 0

try:
    while True:
        sent_data = [ord(char) for char in "hello"]
        print(f"Sending: {sent_data}")
        spi.xfer2(sent_data)
        print(f"Sent: {sent_data}")
        time.sleep(0.5)
finally:
    spi.close()
