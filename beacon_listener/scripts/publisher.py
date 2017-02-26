
class PublishingWrapper:
    """Responsible for filtering beacons and publishing they state"""

    def __init__(self, pub=None, container=None, adv_filter=None):
        self.__publisher = pub
        self.__container = container
        self.__filter = adv_filter

    def publish(self):
        publish_list = self.__filter.do_filter(self.__container)
        # print  len(publish_list)
        for item in publish_list:
            self.__publisher.publish(item)

