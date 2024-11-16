import smbus2
import time
from file_handler import FileHandler
# I2C bus and slave address
I2C_BUS = 1
ESP32_SLAVE_ADDR = 0x08

# Create I2C bus instance
bus = smbus2.SMBus(I2C_BUS)

# Global variables
mode_bot = "NULL"
mode_joystick = "NULL"
analogval = "NULL"
dogmode_handler = FileHandler('Dogmode.txt')
joystick_handler = FileHandler('joystick.txt')

def get_mode_string(mode_esp):
    """Convert mode_esp number to corresponding string"""
    mode_map = {
        1: "Scout",
        2: "Sentry",
        3: "Follow",
        None: "Unknown"
    }
    return mode_map.get(mode_esp, "Unknown")

def write_to_files(mode_bot, mode_joystick, analogval):
    try:
        # Write to joystick.txt
        joystick_data = f"Bot Mode: {mode_bot}\nJoystick Mode: {mode_joystick}\nAnalog Value: {analogval}"
        joystick_handler.write_with_priority(joystick_data, 'espnow', priority=2)
        
        # Write to Dogmode.txt with higher priority
        dogmode_data = f"Mode: {mode_bot}"
        dogmode_handler.write_with_priority(dogmode_data, 'espnow', priority=2)
            
    except Exception as e:
        print(f"Error writing to files: {e}")

def receive_data():
    try:
        total_bytes = 32
        chunk_size = 32
        received_data = []

        while total_bytes > 0:
            chunk = bus.read_i2c_block_data(ESP32_SLAVE_ADDR, 0, min(chunk_size, total_bytes))
            received_data.extend(chunk)
            total_bytes -= len(chunk)

        received_bytes = bytes(received_data)
        received_string = received_bytes.decode('utf-8', errors='ignore')

        print(f"Raw Received Data: {received_string}")

        try:
            # Split the received string and convert mode_esp to integer
            mode_esp_str, mode_joystick, analogval = received_string.split(',')
            mode_esp = int(mode_esp_str.strip())
            mode_joystick = mode_joystick.strip()
            analogval = analogval.strip()

            # Write the received values to both files
            write_to_files(mode_esp, mode_joystick, analogval)

            print(f"Parsed Data - Mode ESP: {mode_esp} ({get_mode_string(mode_esp)}), "
                  f"Joystick Mode: {mode_joystick}, Analog Value: {analogval}")
            return mode_esp, mode_joystick, analogval

        except Exception as parse_error:
            print(f"Error parsing received data: {parse_error}")
            return None, None, None

    except Exception as e:
        print(f"Error receiving data from ESP32: {e}")
        return None, None, None

if __name__ == "__main__":
    while True:
        mode_esp, mode_joystick, analogval = receive_data()

        if all(x is not None for x in [mode_esp, mode_joystick, analogval]):
            print(f"Data received - Mode ESP: {mode_esp} ({get_mode_string(mode_esp)}), "
                  f"Joystick Mode: {mode_joystick}, Analog Value: {analogval}")

        time.sleep(0.5)