from bluepy.btle import DefaultDelegate
from beacon_container import BeaconContainer
import rospy



class BeaconScanDelegate(DefaultDelegate):
    """Delegate to handle bluepy Scanner events"""
    def setContainer(self, container):
        self.__container = container

    def handleDiscovery(self, scanEntry, isNewDev, isNewData):
        rospy.loginfo("Discovery of device %s, RSSI: %s", scanEntry.addr, scanEntry.rssi)
        self.__container.upsert(scanEntry)
