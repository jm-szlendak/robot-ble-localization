import struct


class BeaconPublishingWrapper:
    """Responsible for filtering beacons and publishing they state"""

    def __init__(self, pub=None, container=None, adv_filter=None):
        self.__publisher = pub
        self.__container = container
        self.__filter = adv_filter

    def publish(self):
        publish_list = self.__filter.do_filter(self.__container)
        print len(publish_list)


class BeaconLocalizationFilter:
    """Filters iterable of beacons to remove non-system and outdated ones"""

    def __init__(self, accepted_group, accepted_manufacturer):
        self.__accepted_manufacturer = accepted_manufacturer
        self.__accepted_group_id = accepted_group
        pass

    def do_filter(self, iterable):
        filtered = []
        for x in iterable:
            if not x.scan_data[255]:
                continue
            # TODO: check packet size also
            manufacturer_code = struct.unpack('H', x.scan_data[255][0:2])[0]
            if manufacturer_code != self.__accepted_manufacturer:
                continue
            group_id = struct.unpack('I', x.scan_data[255][2:6])[0]
            if group_id != self.__accepted_group_id:
                continue

            filtered.append(x)
        return filtered
