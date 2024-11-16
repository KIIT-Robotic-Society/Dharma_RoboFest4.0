import smbus
import time

# I2C bus (1 for Raspberry Pi)
bus = smbus.SMBus(1)

# ESP32 I2C address
ESP32_ADDR = 0x32  # Make sure this matches the address in your ESP32 code

# Sample data to send
mode = "2"
mode_joystick = "3"
analogdata = "4"
failsafe = "12"

def send_data(mode, mode_joystick, analogdata, failsafe):
    # Prepare data string to send
    data_string = f"{mode},{mode_joystick},{analogdata},{failsafe}"
    data_bytes = data_string.encode('utf-8')  # Convert the string to bytes

    # Send data in chunks of 32 bytes
    for i in range(0, len(data_bytes), 32):  # Chunk size should not exceed 32 bytes
        chunk = data_bytes[i:i + 32]
        bus.write_i2c_block_data(ESP32_ADDR, 0, list(chunk))  # Send chunk
        print(f"Data sent to ESP32: {chunk.decode('utf-8', errors='ignore')}")
        time.sleep(0.1)  # Short delay between chunks if needed

def main():
    while True:
        # Send data every 2 seconds
        send_data(mode, mode_joystick, analogdata, failsafe)
        time.sleep(2)

if __name__ == "__main__":
    main()
