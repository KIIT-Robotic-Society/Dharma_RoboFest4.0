import spidev
import time
import RPi.GPIO as GPIO

# Set up GPIO for Chip Select (CS)
CS_PIN = 8  # Use GPIO pin 8 for CS
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)

# Set up SPI communication
spi = spidev.SpiDev()
spi.open(0, 0)  # Use SPI bus 0 and chip select 0
spi.max_speed_hz = 50000  # Set SPI clock speed

def send_message(message):
    # Convert the string to a byte array and pad if needed
    data_out = [ord(c) for c in message] + [ord(' ')] * (5 - len(message))
    
    GPIO.output(CS_PIN, GPIO.LOW)  # Activate CS
    spi.xfer2(data_out)  # Send data
    GPIO.output(CS_PIN, GPIO.HIGH)  # Deactivate CS
    print(f"Sent: {message}")

def receive_message():
    GPIO.output(CS_PIN, GPIO.LOW)  # Activate CS
    buffer = spi.xfer2([0] * 5)  # Receive data
    GPIO.output(CS_PIN, GPIO.HIGH)  # Deactivate CS
    
    return ''.join([chr(b) for b in buffer]).strip()  # Return received message

try:
    while True:
        send_message("hello")  # Send message
        response = receive_message()  # Receive response
        print(f"Received from Pico: {response}")
        time.sleep(1)  # Delay between communications

except KeyboardInterrupt:
    print("Interrupted! Closing SPI connection.")
finally:
    spi.close()
    GPIO.cleanup()
