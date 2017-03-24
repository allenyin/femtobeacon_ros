#!/usr/bin/env python

# Serial
from time import sleep
import serial

# ROS
import rospy
from femtobeacon_ros.msg import femtoBeaconMsg


def connectDongle():

    # Declare arduino serial connections
    portName = '/dev/ttyACM0'   # Match Arduino Dongle port
    baudrate = 115200           # Match Arduino Dongle baudrate

    print "Openning dongle connection..."
    portOpened = False 
    while not portOpened:
        try:
            dongle = serial.Serial(port=portName, baudrate=baudrate,
                                   parity=serial.PARITY_NONE,
                                   stopbits=serial.STOPBITS_ONE,
                                   bytesize=serial.EIGHTBITS)
            portOpened = True
        except serial.serialutil.SerialException:
            print "Dongle not connected"
            sleep(3)

    if dongle.isOpen():
        print "Dongle openned correctly"
        sleep(3)
        return dongle

def makeMessage(data):
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

    femtoBeaconMsg should match these fields
    '''
    msg = femtoBeaconMsg()
    msg.header.stamp = rospy.Time.now()
    split_data = data.split(',')
    if len(split_data) < 8:
        print 'Received data is: ', data
        print 'Nothing from femtoBeacon yet!'
        return None

    msg.APP_PANID = int(split_data[0])
    msg.APP_CHANNEL = int(split_data[1])
    msg.APP_ADDRESS = int(split_data[2])
    msg.SRC_ADDR = int(split_data[3])
    msg.timestamp = int(split_data[4])
    msg.yaw = float(split_data[5])
    msg.pitch = float(split_data[6])
    msg.roll = float(split_data[7])
    return msg

def femtobeacon_talker():
    dongle = connectDongle()
    pub = rospy.Publisher('femtobeacon', femtoBeaconMsg, queue_size=10)
    rospy.init_node('femtobeacon_dongle', anonymous=False)  # should only be one dongle
    beaconFound = False
    while not rospy.is_shutdown():
        data = dongle.readline()[:-2]
        msg = makeMessage(data)
        if msg is not None:
            if not beaconFound:
                print 'Found beacon!'
                beaconFound = True
            #rospy.loginfo(msg)
            pub.publish(msg)
            #print msg.header.stamp, msg.yaw

if __name__ == '__main__':
    try:
        femtobeacon_talker()
    except rospy.ROSInterruptException:
        pass
