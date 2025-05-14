import serial

# Define serial port and baud rate (adjust as needed)
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200  # Adjust based on your sensor's specifications

def read_ft_sensor_x():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            ser.flush()  # Clear any existing data in the buffer
            
            # Send a command if required (depends on your sensor's protocol)
            # ser.write(b'YOUR_COMMAND\n')  # Uncomment and modify if needed
            
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    print("Raw Data:", line)  # Debugging: Print received data
                    
                    try:
                        x_force = float(line.split()[0])  # Assuming first value corresponds to X-axis
                        print(f"X-Axis Force: {x_force} N")
                    except (ValueError, IndexError):
                        print("Invalid data format received.")
    
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    read_ft_sensor_x()
