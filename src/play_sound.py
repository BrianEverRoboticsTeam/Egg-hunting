#!/usr/bin/python

import rospy
import numpy as np
from kobuki_msgs.msg import Sound

rospy.init_node('play_sound')
pub = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=1)
sound = Sound()
sound.value = 0

print "test"
print(sound)
pub.publish(sound)

# rospy.spin()
