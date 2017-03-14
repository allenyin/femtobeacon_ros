#!/usr/bin/env python

# Serial
from time import sleep
import serial

def connectDongle():

    # Declare arduino serial connections
    portName = '/dev/ttyACM0'   # Match Arduino Dongle port
    baudrate = 115200           # Match Arduino Dongle baudrate

    print "Openning dongle connection..."
    dongle = serial.Serial(port=portName, baudrate=baudrate,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS)
    if dongle.isOpen():
        print "Dongle openned correctly"
        time.sleep(3)
        return dongle

def parseData(data):
    '''
    Data format from dongle is:
        APP_PANID,APP_CHANNEL,APP_ADDRESS,SRC_ADDR,timestamp,yaw,pitch,roll
    Values separated by ","

    APP_PANID = Dongle device ID (set by user)
    APP_CHANNEL = wireless channel
    APP_ADDRESS = Dongle device addres (set by user, should be 1 for dongle. Then increment for each coin)
    SRC_ADDR = APP_ADDRESS of the coin we are receiving from
    timestamp = Data timestamped on the sensor, in [ms]
    yaw = In degrees, [-180, 180]
    pitch = In degrees, [-180, 180]
    roll = In degrees, [-90, 90]

    Returns a dictionary with these fields
    '''
    split_data = data.split(',')
    output = {}
    output['APP_PANID'] = split_data[0]
    output['APP_CHANNEL'] = int(split_data[1])
    output['APP_ADDRESS'] = int(split_data[2])
    output['SRC_ADDR'] = int(split_data[3])
    output['timestamp'] = int(split_data[4])
    output['yaw'] = float(split_data[5])
    output['pitch'] = float(split_data[6])
    output['roll'] = float(split_data[7])
    return output

while True:
    data = dongle.readline()[:-2]
    if data:
        print parseData(data) 

