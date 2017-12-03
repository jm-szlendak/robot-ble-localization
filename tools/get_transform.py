#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import rospy
import tf

if __name__ == "__main__":
    # init ros
    rospy.init_node("get_tf")

    rospy.sleep(1)

    listener = tf.TransformListener()

    (trans, rot) = listener.lookupTransform('batman/base_link', '/map', rospy.Time(0))


    print trans
