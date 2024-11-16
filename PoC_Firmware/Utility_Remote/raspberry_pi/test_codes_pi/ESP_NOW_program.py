import smbus2
import time

# I2C bus and slave address
I2C_BUS = 1  # Typically 1 for Raspberry Pi
ESP32_SLAVE_ADDR = 0x78  # Slave I2C address (ESP32 configured as slave)

# Create I2C bus instance
bus = smbus2.SMBus(I2C_BUS)

# Global variables
mode_esp = -1
mode_raspberry=4
poc_battery = 8
remote_battery = 98
enemy=127
enemy_string=str(enemy)+"0"
enemy_detected =int(enemy_string)

def send_data():
    try:
        # Prepare data string to send
        data_string = f"{mode_raspberry},{poc_battery},{remote_battery},{enemy_detected}"
        data_bytes = data_string.encode('utf-8')  # Convert the string to bytes

        # Send data in chunks of 32 bytes
        for i in range(0, len(data_bytes), 32):  # Chunk size should not exceed 32 bytes
            chunk = data_bytes[i:i + 32]
            bus.write_i2c_block_data(ESP32_SLAVE_ADDR, 0, list(chunk))  # Send chunk
            print(f"Data sent to ESP32: {chunk.decode('utf-8', errors='ignore')}")
            time.sleep(0.1)  # Short delay between chunks if needed

    except Exception as e:
        print(f"Error sending data to ESP32: {e}")

def receive_data():
    try:
        total_bytes = 44  # Expected total bytes of data
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

        # Parsing the data assuming a format like:
        # "Pitch,Roll,Yaw,Latitude,Longitude,Satellites,BPM,AvgBPM,Mode,Failsafe,Battery"
        try:
            # Split the string by commas to extract individual data values
            data_values = received_string.split(',')

            # Extract each value and cast to appropriate types
            pitch = int(data_values[0])
            roll = int(data_values[1])
            yaw = int(data_values[2])
            latitude = int(data_values[3])
            longitude = int(data_values[4])
            satellites = int(data_values[5])
            bpm = int(data_values[6])
            avg_bpm = int(data_values[7])
            mode = int(data_values[8])
            failsafe = int(data_values[9])
            battery = int(data_values[10])

            # Print the parsed values
            print(f"Parsed Data - Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}, Latitude: {latitude}, "
                  f"Longitude: {longitude}, Satellites: {satellites}, BPM: {bpm}, Avg BPM: {avg_bpm}, "
                  f"Mode: {mode}, Failsafe: {failsafe}, Battery: {battery}")

            return pitch, roll, yaw, latitude, longitude, satellites, bpm, avg_bpm, mode, failsafe, battery

        except Exception as parse_error:
            print(f"Error parsing received data: {parse_error}")
            return None, None, None, None, None, None, None, None, None, None, None

    except Exception as e:
        print(f"Error receiving data from ESP32: {e}")
        return None, None, None, None, None, None, None, None, None, None, None

# Example of calling the send_data and receive_data functions
# Example of calling the send_data and receive_data functions
if __name__ == "__main__":
    while True:
        # Send data to ESP32
        send_data()

        # Receive and parse the data
        pitch, roll, yaw, latitude, longitude, satellites, bpm, avg_bpm, mode_esp, failsafe, battery = receive_data()
        if pitch is not None:
            print(f"Data received - Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}, Latitude: {latitude}, "
                  f"Longitude: {longitude}, Satellites: {satellites}, BPM: {bpm}, Avg BPM: {avg_bpm}, "
                  f"ModeESP: {mode_esp}, Failsafe: {failsafe}, Battery: {battery}")

        # Check if mode_esp is not None before comparing with mode_raspberry
        if mode_esp is not None and mode_esp != mode_raspberry:
            mode_raspberry = mode_esp


