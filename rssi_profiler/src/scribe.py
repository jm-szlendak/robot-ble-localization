#!/usr/bin/env python
import sys
import os.path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from beacon_msgs.srv import RSSIProfile
import rospy
import tf

def multi_measure(distance, count, bid, output_filename, service):
    try:
        with open(output_filename, 'a') as outfile:

            rospy.loginfo('Calling service for %s measurements' % count)
            measure = service.call(distance, count, bid)
            line = str(distance)
            line += '\t'
            line += str(measure.avg_rssi)
            line += '\t'
            line += str(measure.std_dev)
            line += '\n'
            outfile.write(line)
    except IOError:
        print 'Unable to open file'
        exit(1)


def single_measure(distance, count, bid, output_filename, service):
    try:
        with open(output_filename, 'w') as outfile:
            outfile.truncate()
            rospy.loginfo('Calling service for %s measurements' % count)
            measure = service.call(distance, count, bid)
            outfile.writelines([str(item) + '\n' for item in measure.measurements])
            # outfile.write(line)
    except IOError:
        print 'Unable to open file'
        exit(1)


#
# def auto_measure(tf_listener):
#     try:
#         tf_listener.lookupTransform('/')

def main():
    # init ros
    rospy.init_node("rssi_scribe")

    args = rospy.myargv(argv=sys.argv)

    if len(args) != 6:
        print "Usage:"
        print "python scribe.py [distance] [measurements count] [beacon id] [output filename] [single | multi]"
        exit(1)

    distance = float(args[1])
    count = int(args[2])
    bid = args[3]
    outfile_name = args[4]
    mode = args[5]

    rospy.loginfo("Starting RSSI Scribe")

    rospy.wait_for_service('/batman/beacon_localization/rssi_profile')
    rssi_profile = rospy.ServiceProxy('/batman/beacon_localization/rssi_profile', RSSIProfile)

    rospy.loginfo('Service rssi_profile ready')


    if mode == 'single':
        single_measure(distance, count, bid, outfile_name, rssi_profile)
    elif mode == 'multi':
        multi_measure(distance, count, bid, outfile_name, rssi_profile)
    else:
        raise Exception(
            'Invalid mode: use "single" for column-oriented file or "multi" for many measurements (append mode)')


if __name__ == "__main__":
    main()
