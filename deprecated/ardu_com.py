import serial
import time
import math

def calculate_crc(data):
    crc = 0
    for byte in data[:-1]:  # Exclude the last byte (CRC byte)
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07  # Polynomial 0x07 for CRC-8
            else:
                crc <<= 1
            crc &= 0xFF  # Ensure CRC remains within 8 bits
    return crc

def send_data_to_com_port(port, signal, brightness_values):
    # try:
        # Ensure brightness_values has exactly 5 bytes
        if len(brightness_values) != 6:
            raise ValueError("Brightness values must be exactly 6 bytes.")
        
        # Construct the data packet
        # print(f"Data to send: {list(data)}")
        data = bytearray([signal]) + bytearray(brightness_values)
        crc = calculate_crc(data)
        data.append(crc)

        t0 = time.time()  # Start time for the loop

        # Open the COM port with baud rate 115200
        with serial.Serial(port, baudrate=115200, timeout=1) as ser:
            while True:
                # Send the data

                # set brightness values with sine wave
                current_time = time.time() - t0
                fequency = 0.05  # Frequency of the sine wave modulation
                # Modulate brightness values using a sine wave
                # Example: Modulate the first brightness value with a sine wave
                brightness_values = [
                    int(30 + 112.5 * (math.sin(2 * math.pi * fequency * current_time) + 1)) for _ in range(6)
                ]


                data = bytearray([signal]) + bytearray(brightness_values)
                crc = calculate_crc(data)
                data.append(crc)
                ser.write(data)
                print(f"Data sent to {port}: {data}")
                
                # Read response
                response = ser.read(8)  # Expecting 8 bytes
                if len(response) != 8:
                    print("Invalid response length.")
                    continue

                if len(response) == 8:
                    received_crc = response[-1]
                    # print("checking crc")
                    calculated_crc = calculate_crc(response)
                    if received_crc != calculated_crc:
                        print("CRC mismatch in response.")
                        continue
                
                # Parse response
                digital_output = response[0]
                brightness_values = response[1:7]
                switch_states = response[6]
                
                print(f"Digital Output: {digital_output}")
                print(f"Brightness Values: {list(brightness_values)}")
                print(f"Switch States: {bin(switch_states)}")
                
                # Wait for 100ms to achieve 10Hz
                time.sleep(0.1)
    # except Exception as e:
    #     print(f"Error: {e}")

if __name__ == "__main__":
    # Example usage
    com_port = "COM4"  # Replace with your COM port
    signal = 1  # Example signal (1 or 0)
    brightness_values = [0xFF, 0xFF, 0x80, 0x40, 0x20, 0x00]  # Example brightness values
    send_data_to_com_port(com_port, signal, brightness_values)