from bluepy.btle import DefaultDelegate
import time
import copy


class AdvertisingPacket:
    """Beacon entity"""

    def __init__(self, scan_entry):
        self.addr = scan_entry.addr
        self.updated_at = time.time()
        self.raw = scan_entry.rawData
        self.rssi = scan_entry.rssi
        self.scan_data = copy.deepcopy(scan_entry.scanData)


class BeaconScanDelegate(DefaultDelegate):
    """Delegate to handle bluepy Scanner events"""

    def __init__(self, container=None):
        DefaultDelegate.__init__(self)
        self.__container = container

    def set_container(self, container):
        self.__container = container

    def handleDiscovery(self, scanEntry, isNewDev, isNewData):
        # rospy.loginfo("Discovery of device %s, RSSI: %s", scanEntry.addr, scanEntry.rssi)
        packet = AdvertisingPacket(scanEntry)
        self.__container.insert(packet.addr, packet)
