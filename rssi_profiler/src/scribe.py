#!/usr/bin/env python
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.srv import RSSIProfile

import rospy

if __name__ == "__main__":

    # init ros
    rospy.init_node("rssi_scribe")

    args = rospy.myargv(argv=sys.argv)

    distance = float(args[1])
    count = int(args[2])
    outfile_name = args[3]

    if len(args) != 4:
        print "Usage:"
        print "python scribe.py [distance] [measurements count] [output filename]"
        exit(1)

    rospy.loginfo("Starting RSSI Scribe")

    rospy.wait_for_service('/beacon_localization/rssi_profile')
    rssi_profile = rospy.ServiceProxy('/beacon_localization/rssi_profile', RSSIProfile)

    rospy.loginfo('Service rssi_profile ready')

    try:
        with open(args[3], 'a') as outfile:

            rospy.loginfo('Calling service for %s measurements' % count)
            measure = rssi_profile.call(distance, count)
            line = str(distance)
            line += '\t'
            line += '\t'.join([str(item) for item in measure.measurements])
            line += '\t'
            line += str(measure.avg_rssi)
            line += '\t'
            line += str(measure.std_dev)
            line += '\n'
            outfile.write(line)
    except IOError:
        print 'Unable to open file'
        exit(1)

    # try:
        # result = rssi_profile.call()


