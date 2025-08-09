import threading
import time
import math
import serial
import logging

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

class ArduinoWorker(threading.Thread):
    def __init__(self, port="COM4", baudrate=115200, tick=0.01):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.tick = tick
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()
        
        # Communication parameters
        self.signal = 1
        self.brightness_values = [0xFF, 0xFF, 0x80, 0x40, 0x20, 0x00]
        
        # Status data
        self.status = {
            'connected': False,
            'digital_output': 0,
            'received_brightness': [0] * 6,
            'switch_states': 0,
            'last_update': 0,
            'error_count': 0
        }
        
        # Serial connection
        self.ser = None
        self.t0 = 0
        
    def connect(self):
        """Connect to Arduino"""
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=1)
            with self.lock:
                self.status['connected'] = True
            logging.info(f"Serial connected to {self.port}")
            return True
        except Exception as e:
            logging.error(f"Failed to connect to Serial: {e}")
            with self.lock:
                self.status['connected'] = False
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        with self.lock:
            self.status['connected'] = False
        logging.info("Serial disconnected")
    
    def set_brightness_values(self, values):
        """Set brightness values (6 bytes)"""
        if len(values) == 6:
            with self.lock:
                self.brightness_values = values.copy()
    
    def set_led_brightness(self, led_index, brightness):
        """Set specific LED brightness (led_index: 0-5, brightness: 0-255)"""
        if 0 <= led_index <= 5 and 0 <= brightness <= 255:
            with self.lock:
                self.brightness_values[led_index] = brightness
    
    def set_all_leds(self, brightness):
        """Set all LEDs to same brightness (brightness: 0-255)"""
        if 0 <= brightness <= 255:
            with self.lock:
                self.brightness_values = [brightness] * 6
    
    def set_signal(self, signal):
        """Set signal value (0 or 1)"""
        with self.lock:
            self.signal = signal
    
    def get_status(self):
        """Get current status"""
        with self.lock:
            return self.status.copy()
    
    def stop(self):
        """Stop the worker thread"""
        self.stop_evt.set()
    
    def run(self):
        """Main thread loop"""
        self.t0 = time.time()
        
        while not self.stop_evt.is_set():
            if not self.status['connected']:
                time.sleep(self.tick)
                continue
                
            try:
                # Construct the data packet
                with self.lock:
                    signal = self.signal
                    brightness_data = self.brightness_values.copy()
                
                data = bytearray([signal]) + bytearray(brightness_data)
                crc = calculate_crc(data)
                data.append(crc)
                
                # Send data
                if self.ser and self.ser.is_open:
                    self.ser.write(data)
                    logging.debug(f"Data sent to {self.port}: {list(data)}")
                    
                    # Read response
                    response = self.ser.read(8)  # Expecting 8 bytes
                    
                    if len(response) == 8:
                        received_crc = response[-1]
                        calculated_crc = calculate_crc(response)
                        
                        if received_crc == calculated_crc:
                            # Parse response
                            digital_output = response[0]
                            brightness_values = list(response[1:7])
                            switch_states = response[6]
                            
                            # Update status
                            with self.lock:
                                self.status.update({
                                    'digital_output': digital_output,
                                    'received_brightness': brightness_values,
                                    'switch_states': switch_states,
                                    'last_update': time.time(),
                                    'connected': True
                                })
                            
                            logging.debug(f"Digital Output: {digital_output}")
                            logging.debug(f"Brightness Values: {brightness_values}")
                            logging.debug(f"Switch States: {bin(switch_states)}")
                        else:
                            logging.warning("CRC mismatch in response")
                            with self.lock:
                                self.status['error_count'] += 1
                    else:
                        logging.warning(f"Invalid response length: {len(response)}")
                        with self.lock:
                            self.status['error_count'] += 1
                            
            except Exception as e:
                logging.error(f"Serial communication error: {e}")
                with self.lock:
                    self.status['error_count'] += 1
                    self.status['connected'] = False
                
                # Try to reconnect
                self.disconnect()
                time.sleep(1)  # Wait before retry
                self.connect()
            
            time.sleep(self.tick)
        
        # Cleanup on exit
        self.disconnect()
