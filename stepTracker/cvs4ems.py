import sys
import serial
import csv
import time
import atexit

# defined constants for serial coms
SERIAL_PORT = "COM4"
BAUD_RATE = 115200

# init serial connection
cereal = serial.Serial(SERIAL_PORT,BAUD_RATE)

# init lists to store data
adxlVec = []

# function to read arduino data
def readArduino():
    value = cereal.readline().decode('utf-8').strip()
    adxlVec.append(value)
    
    # debug line
    print(value)

def writeCVS(adxlVec):
    with open('ADXL_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['ADXL Vector'])
        for vec in adxlVec:
            writer.writerow([vec])



@atexit.register
def on_close():
    writeCVS(adxlVec)

while True:
    readArduino()
    time.sleep(0.01)
