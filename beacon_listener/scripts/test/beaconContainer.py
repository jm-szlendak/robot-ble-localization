from unittest import TestCase
from time import time
from bluepy.btle import ScanEntry
from beacon_listener.scripts.scan import AdvertisingPacket
from beacon_listener.scripts.container import Container

scan_entry = ScanEntry('7e:96:b0:dc:fc:95', 0)
scan_entry2 = ScanEntry('7e:96:b0:dc:fc:96', 0)
packet = AdvertisingPacket(scan_entry)
packet2 = AdvertisingPacket(scan_entry2)


class TestBeaconContainer(TestCase):

    def test_clean(self):
        container = Container()

        packet.updated_at = 0
        packet2.updated_at = time()
        container.insert(packet.addr, packet)
        container.insert(packet2.addr, packet2)
        container.clean()
        self.assertIsNone(container.get('7e:96:b0:dc:fc:95'))
        self.assertIsNotNone(container.get('7e:96:b0:dc:fc:96'))

    def test_iter(self):
        container = Container()
        container.insert(packet.addr, packet)
        container.insert(packet2.addr, packet2)
        # print container
        for item in container:
            self.assertIsNotNone(item)
            # self.assertIsNotNone(b)