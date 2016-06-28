

class BeaconPublishingWrapper:
    """Responsible for filtering beacons and publishing they state"""

    def __init__(self, pub, container, filter):
        self.__publisher = pub
        self.__container = container
        self.__filter = filter

    def publish(self):
        publish_list = filter.do_filter(self.__container)


class BeaconLocalizationFilter:
    """Filters iterable of beacons to remove non-system and outdated ones"""

    def do_filter(self, iterable):
        for x in iterable:
            
            #parse packet etc
            pass
