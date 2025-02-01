import spidev
import time

# Set up SPI communication
spi = spidev.SpiDev()
spi.open(0, 0)  # Use SPI bus 0 and chip select 0 (CS0)
spi.max_speed_hz = 50000  # Set SPI clock speed

def send_message(message):
    # Convert the string to a byte array
    data_out = [ord(c) for c in message]
    # Pad with spaces if the message is less than 5 characters
    data_out += [ord(' ')] * (5 - len(data_out))
    
    # Send data over SPI
    spi.xfer2(data_out)
    print(f"Sent: {message}")

def receive_message():
    # Buffer to receive the response (same size as the message buffer)
    buffer = spi.xfer2([0] * 5)  # Sending dummy data (0s) to receive the message
    
    # Convert received byte array back to a string
    received_message = ''.join([chr(b) for b in buffer])
    return received_message.strip()  # Remove any trailing spaces

try:
    while True:
        # Send "hello" to the Pico
        send_message("hello")
        # Receive the response from the Pico
        response = receive_message()
        print(f"Received from Pico: {response}")
        # Small delay before next communication
        time.sleep(1)

except KeyboardInterrupt:
    print("Interrupted! Closing SPI connection.")
finally:
    # Close SPI connection when done
    spi.close()
