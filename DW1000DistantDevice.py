"""
    DW1000Device.py
    This python module maintains remote devices to get activity and other information
    (range, rx_power, fp_power, quality, last received time)
"""
import random
import time
import DW1000Constants as C
import DW1000Time
import DW1000Mac


class DW1000DistantDevice(object):
    timePollSent = DW1000Time
    timePollReceived = DW1000Time
    timePollAckSent = DW1000Time
    timePollAckReceived = DW1000Time
    timeRangeSent = DW1000Time
    timeRangeReceived = DW1000Time

    _own_address = bytearray([0, 0, 0, 0, 0, 0, 0, 0])
    _short_address = bytearray([0, 0])
    _activity = 0
    _reply_delay_time_us = 0
    _index = 0

    _range = 0
    _rx_power = 0
    _fp_power = 0
    _quality = 0

    def __init__(self, device_address=None, short_address=None):
        if device_address is None:
            self.random_address()
            return
        else:
            self.set_address(device_address)
        if short_address is None:
            self.random_short_address()
        else:
            self.set_short_address(device_address)

    def __del__(self):
        return

    # Setters
    def set_reply_time(self, reply_delay_time_us):
        self._reply_delay_time_us = reply_delay_time_us

    def set_address(self, device_address):
        self._own_address = device_address
        #DW1000.converToByte(deviceAddress, _ownAddress)

    def set_short_address(self, short_address):
        self._short_address = short_address
        # memcpy(_shortAddress, deviceAddress, 2)
    
    def set_range(self, given_range):
        self._range = given_range

    def set_rx_power(self, rx_power):
        self._rx_power = rx_power

    def set_fp_power(self, fp_power):
        self._fp_power = fp_power

    def set_quality(self, quality):
        self._quality = quality

    # getter
    def get_byte_address(self):
        return self._own_address

    def get_byte_short_address(self):
        return self._short_address

    def get_short_address(self):
        return self._short_address

    def is_address_equal(self, device):
        if self.get_byte_address == device.get_byte_address:
            return True
        else:
            return False

    def is_short_address_equal(self, device):
        if self.get_byte_short_address == device.get_byte_short_address:
            return True
        else:
            return False

    def get_range(self):
        return self._range

    def get_rx_power(self):
        return self._rx_power

    def get_fp_power(self):
        return self._fp_power

    def get_quality(self):
        return self._quality

    def random_short_address(self):
        self._short_address[0] = random.uniform(0, 255)
        self._short_address[1] = random.uniform(0, 255)

    def random_address(self):
        self._own_address[0] = random.uniform(0, 255)
        self._own_address[1] = random.uniform(0, 255)
        self._own_address[2] = random.uniform(0, 255)
        self._own_address[3] = random.uniform(0, 255)
        self._own_address[4] = random.uniform(0, 255)
        self._own_address[5] = random.uniform(0, 255)
        self._own_address[6] = random.uniform(0, 255)
        self._own_address[7] = random.uniform(0, 255)

    def note_activity(self):
        self._activity = int(round(time.time()*1000))

    def is_inactive(self):
        if (int(round(time.time()*1000))-self._activity) > C.INACTIVITY_TIME:
            self._activity = int(round(time.time()*1000))
            return True
        else:
            return False
        