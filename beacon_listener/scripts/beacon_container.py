
class BeaconContainer:
    """Container class for storing active devices. Inactive devices may be cleaned up"""

    def __init__(self):
        self.__set = {}

    def clean(self):
        pass

    def insert(self, dev):
        pass

    def upsert(self, dev):
        self.insert(dev)

    def get(self, addr):
        pass