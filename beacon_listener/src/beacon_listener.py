import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from scripts.scan import BeaconScanDelegate
from bluepy.btle import Scanner



if __name__ == "__main__":
    scanDelegate = BeaconScanDelegate()
    scanner = Scanner().withDelegate(scanDelegate)
    scanner.start()