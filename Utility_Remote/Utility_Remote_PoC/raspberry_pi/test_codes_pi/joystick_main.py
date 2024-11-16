import smbus2
import time

# I2C bus and slave address
I2C_BUS = 1  # Typically 1 for Raspberry Pi
ESP32_SLAVE_ADDR = 0x08  # Slave I2C address (ESP32 configured as slave)

# Create I2C bus instance
bus = smbus2.SMBus(I2C_BUS)

# Global variables
mode_bot = "NULL"
mode_joystick = "NULL"
analogval = "NULL"

def receive_data():
    try:
        total_bytes = 32  # Expected total bytes of data (adjust based on your actual data size)
        chunk_size = 32  # Max 32 bytes per block
        received_data = []

        # Read in chunks and append to the list
        while total_bytes > 0:
            chunk = bus.read_i2c_block_data(ESP32_SLAVE_ADDR, 0, min(chunk_size, total_bytes))
            received_data.extend(chunk)
            total_bytes -= len(chunk)

        # Convert the received raw bytes to a string
        received_bytes = bytes(received_data)
        received_string = received_bytes.decode('utf-8', errors='ignore')  # Decode using UTF-8, ignoring bad characters

        print(f"Raw Received Data: {received_string}")

        # Split the received string by comma to get mode_bot, mode_joystick, and analogval
        try:
            mode_bot, mode_joystick, analogval = received_string.split(',')
            mode_bot = mode_bot.strip()  # Remove any surrounding whitespace
            mode_joystick = mode_joystick.strip()  # Remove any surrounding whitespace
            analogval = analogval.strip()  # Remove any surrounding whitespace

            # Print the parsed values
            print(f"Parsed Data - Mode Bot: {mode_bot}, Joystick Mode: {mode_joystick}, Analog Value: {analogval}")

            return mode_bot, mode_joystick, analogval

        except Exception as parse_error:
            print(f"Error parsing received data: {parse_error}")
            return None, None, None

    except Exception as e:
        print(f"Error receiving data from ESP32: {e}")
        return None, None, None

# Main logic
if __name__ == "__main__":
    while True:
        # Receive and parse the data
        mode_bot, mode_joystick, analogval = receive_data()

        if mode_bot is not None and mode_joystick is not None and analogval is not None:
            print(f"Data received - Mode Bot: {mode_bot}, Joystick Mode: {mode_joystick}, Analog Value: {analogval}")

        # Implement additional logic based on the received data
        time.sleep(0.5)
