import serial
import time

# Configure the serial port and the baud rate
ser = serial.Serial('COM8', 9600)  # Replace 'COM3' with your Arduino's COM port
time.sleep(2)  # Wait for the serial connection to initialize

# Open the output file in append mode
with open('output.txt', 'a') as f:
    while True:
        try:
            if ser.in_waiting > 0:
                # Read a line from the serial port
                data = ser.readline().decode('utf-8', errors='ignore').rstrip()
                print(data)  # Print data to the console
                f.write(data + '\n')  # Write data to the file
        except UnicodeDecodeError:
            # Handle decoding errors
            print("UnicodeDecodeError: Skipping invalid byte sequence")
        except Exception as e:
            # Handle any other exceptions
            print(f"An error occurred: {e}")

