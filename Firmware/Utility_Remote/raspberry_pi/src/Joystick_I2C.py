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

def write_to_files(mode_esp, mode_joystick, analogval):
    try:
        # Convert mode_esp to string representation
        mode_bot_str = get_mode_string(mode_esp)
        
        # Write to joystick.txt
        joystick_data = f"Bot Mode: {mode_bot_str}\nJoystick Mode: {mode_joystick}\nAnalog Value: {analogval}"
        joystick_handler.write_with_priority(joystick_data, 'joystick', priority=1)
        
        # Write to Dogmode.txt with lower priority
        dogmode_data = f"Mode: {mode_bot_str}"
        dogmode_handler.write_with_priority(dogmode_data, 'joystick', priority=1)
            
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
            mode_bot, mode_joystick, analogval = received_string.split(',')
            mode_bot = mode_bot.strip()
            mode_joystick = mode_joystick.strip()
            analogval = analogval.strip()

            # Write the received values to both files
            write_to_files(mode_bot, mode_joystick, analogval)

            print(f"Parsed Data - Mode Bot: {mode_bot}, Joystick Mode: {mode_joystick}, Analog Value: {analogval}")
            return mode_bot, mode_joystick, analogval

        except Exception as parse_error:
            print(f"Error parsing received data: {parse_error}")
            return None, None, None

    except Exception as e:
        print(f"Error receiving data from ESP32: {e}")
        return None, None, None

if __name__ == "__main__":
    while True:
        mode_bot, mode_joystick, analogval = receive_data()

        if mode_bot is not None and mode_joystick is not None and analogval is not None:
            print(f"Data received - Mode Bot: {mode_bot}, Joystick Mode: {mode_joystick}, Analog Value: {analogval}")

        time.sleep(0.5)