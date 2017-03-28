#!/usr/bin/python
import rospy
from std_msgs.msg import String
import serial

decay = 0.5
port = '/dev/ttyACM0'
bitrate = 9600

if __name__ == '__main__':
    rospy.init_node('ultrasonic_node')
    publisher = rospy.Publisher('ultrasonic', String, queue_size=1)
    # rate = rospy.Rate(10)
    last_dist = None
    with serial.Serial(port, bitrate, timeout=1) as ser:
        print 'Ultrasonic sensor started'
        while not rospy.is_shutdown():
            if ser.inWaiting() > 0:
                dist = ser.readline().strip()
                if dist.isdigit():
                    dist = int(dist)
                    if last_dist != None:
                        dist = dist * (1 - decay) + last_dist * decay
                    last_dist = dist
                    dist = str(int(dist))
                    publisher.publish(dist)
                    # rate.sleep()
