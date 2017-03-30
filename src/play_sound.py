#!/usr/bin/python

import rospy
import numpy as np
from kobuki_msgs.msg import Sound

rospy.init_node('play_sound')
pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size = 1)
sound = Sound()
sound.value = 0

while not rospy.is_shutdown():
    pub.publish(sound)
    rospy.sleep(0.3)
    pub.publish(sound)
    rospy.sleep(0.3)
    pub.publish(sound)
    rospy.sleep(0.1)
    pub.publish(sound)
    raw_input("HEY!")
    rospy.sleep(0.01)
