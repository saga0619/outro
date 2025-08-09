#!/usr/bin/env python3
"""
Test script for Arduino worker thread functionality
This script tests the ArduinoWorker class independently
"""

import time
import logging
from ardu_worker import ArduinoWorker

def test_arduino_worker():
    """Test Arduino worker functionality"""
    logging.basicConfig(level=logging.INFO)
    
    logging.info("Testing Arduino Worker Thread...")
    
    # Create Arduino worker instance
    arduino_worker = ArduinoWorker(port="COM4", baudrate=115200, tick=0.1)
    
    try:
        # Try to connect
        if arduino_worker.connect():
            print("✓ Arduino connected successfully")
            
            # Start the worker thread
            arduino_worker.start()
            print("✓ Arduino worker thread started")
            
            # Run for 10 seconds and monitor status
            start_time = time.time()
            while time.time() - start_time < 10:
                status = arduino_worker.get_status()
                print(f"Status: Connected={status['connected']}, "
                      f"Digital Output={status['digital_output']}, "
                      f"Switch States={bin(status['switch_states'])}, "
                      f"Error Count={status['error_count']}")
                time.sleep(1)
            
            # Test setting brightness values
            print("Testing brightness control...")
            arduino_worker.set_brightness_values([0xFF, 0x80, 0x40, 0x20, 0x10, 0x00])
            arduino_worker.set_signal(0)
            time.sleep(2)
            
            arduino_worker.set_signal(1)
            time.sleep(2)
            
        else:
            print("✗ Failed to connect to Arduino")
            print("Note: This is expected if no Arduino is connected to COM4")
            
    except Exception as e:
        print(f"✗ Error during test: {e}")
        
    finally:
        # Clean up
        print("Stopping Arduino worker...")
        arduino_worker.stop()
        arduino_worker.join()
        print("✓ Arduino worker stopped")

if __name__ == "__main__":
    test_arduino_worker()
