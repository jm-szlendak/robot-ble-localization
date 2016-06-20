from bluepy.btle import DefaultDelegate


class BeaconScanDelegate(DefaultDelegate):
    """Delegate to handle bluepy Scanner events"""
    def handleDiscovery(self, scanEntry, isNewDev, isNewData):
        print(scanEntry, isNewDev, isNewData)
