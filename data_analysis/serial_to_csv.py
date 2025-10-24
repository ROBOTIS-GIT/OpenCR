#!/usr/bin/env python3
"""
This script collects incoming data from the serial connection and writes it to a CSV file
"""

import serial
import csv
import time
import signal
import sys

def signal_handler(sig, frame):
    print('\nData collection stopped')
    ser.close()
    sys.exit(0)

# Set up signal handler for clean exit
signal.signal(signal.SIGINT, signal_handler)

# Configure serial connection
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for connection to establish

# Set collection time (in seconds)
COLLECTION_TIME = 120  # 2 minutes
start_time = time.time()

with open('data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    
    print(f"Starting data collection for {COLLECTION_TIME} seconds...")
    
    while time.time() - start_time < COLLECTION_TIME:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                data = line.split(',')
                writer.writerow(data)
                print(line)  # Optional: show data being collected
        except Exception as e:
            print(f"Error: {e}")
            break
    
    print("Data collection completed!")
    ser.close()