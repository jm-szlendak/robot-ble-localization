from beacon_msgs.srv import RSSIProfileResponse
import numpy as np


class RSSIProfileServiceWrapper:
    def __init__(self, data_collector, measurements_count):
        self.__collector = data_collector
        self.__measurements_count = measurements_count

    def handler(self, request):
        # response = RSSIProfileResponse()
        measurements = self.__collector.wait_for_measurements(self.__measurements_count)
        data = np.array([measurement.rssi for measurement in measurements])
        avg = np.average(data)
        std_dev = np.std(data)
        return RSSIProfileResponse(avg_rssi=avg, std_dev=std_dev, count=self.__measurements_count, measurements=data)

