#!/usr/bin/python

import rospy
import numpy as np
from kobuki_msgs.msg import Sound

def main():
    rospy.init_node('play_sound')
    pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size = 1)
    sound = Sound()
    sound.value = 0

    while not rospy.is_shutdown():
        pub.publish(sound)
        rospy.sleep(0.5)


if __name__ == "__main__":
	main()
