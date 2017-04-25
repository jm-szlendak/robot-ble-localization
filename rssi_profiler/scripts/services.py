from beacon_msgs.srv import RSSIProfileResponse, RSSIProfileRequest
import rospy
import numpy as np


class RSSIProfileServiceWrapper:
    def __init__(self, data_collector, measurements_count):
        self.__collector = data_collector
        self.__measurements_count = measurements_count

    def handler(self, request):
        rospy.loginfo("Service rssi_profile called")
        # response = RSSIProfileResponse()
        measurements = self.__collector.wait_for_measurements(request.count, request.bid)
        data = np.array([measurement.rssi for measurement in measurements])
        avg = np.average(data)
        std_dev = np.std(data)
        return RSSIProfileResponse(avg_rssi=avg, std_dev=std_dev,  measurements=data)

