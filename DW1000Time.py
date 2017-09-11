"""
    This python module deal with time/timestamp/distance measurement/management of DW1000
"""

import DW1000Constants as C

class DW1000Time():
    """
        This class manages timestamp
    """

    _timestamp = 0

    def __init__(self, data=None, timestamp=None, index=None):
        if timestamp is None:
            self.timestamp = 0
        self.set_timestamp(data, timestamp, index)

    def __del__(self):
        self._timestamp = 0

    def set_timestamp(self, data, timestamp, index):
        """
        This function sets the specified timestamp into the data that will be sent.

        Args:
                data: The data where you will store the timestamp
                timeStamp = The timestamp's value
                index = The bit from where you will put the timestamp's value

        Returns:
                The data with the timestamp added to it
        """
        for i in range(0, C.LENGTH_TIMESTAMP):
            data[i+index] = int((timestamp >> (i * 8)) & C.MASK_LS_BYTE)

    def get_timestamp(self, data, index):
        """
        This function gets the timestamp's value written inside the specified data and returns it.

        Args:
                data : the data where you want to extract the timestamp from
                index : the index you want to start reading the data from

        Returns:
                The timestamp's value read from the given data.
        """
        timestamp = 0
        for i in range(0, C.LENGTH_TIMESTAMP):
            timestamp |= data[i+index] << (i*8)
        return timestamp

    def wrap_timestamp(self, timestamp):
        """
        This function converts the negative values of the timestamp
        due to the overflow into a correct one.
        Args :
                timestamp : the timestamp's value you want to correct.
        Returns:
                The corrected timestamp's value.
        """
        if timestamp < 0:
            timestamp += C.TIME_OVERFLOW
        return timestamp

    def get_as_float(self):
        """
            This function returns timestamp as microseconds
        """
        self.get_as_micro_seconds()

    def get_as_micro_seconds(self):
        """
            This function returns timestamp as microseconds
        """
        return (self.timestamp % C.TIME_OVERFLOW) * C.TIME_RES

    def get_as_meters(self):
        """
            This function returns travel distance as meters with respect to travel time(timestamp)
        """
        return (self.timestamp%C.TIME_OVERFLOW)*C.DISTANCE_OF_RADIO

    def is_valid_timestamp(self):
        """
            This function check and returns whether timestamp is valid or not
        """
        return 0 <= self.timestamp & self.timestamp <= C.TIME_MAX

'''
    ## Operator overloading

    # assign
    def __setattr__(self, assign):
        if self == assign:
            return self
        self.timestamp = assign.get_timestamp()
        return self

    # add
    def __iadd__(self, add):
        self.timestamp += add.get_timestamp()
        return self

    def __add__(self, add):
        return (self += add)

    # subtract
    def __isub__(self, sub):
        self.timestamp -= sub.get_timestamp()
        return self

    def __sub__(self, sub):
        return (self -= sub)

    # multiply
    def __imul__(self, factor):
        self.timestamp *= factor
        return self

    def __mul__(self, factor):
        return (self *= factor)

    # divide
    def __idiv__(self, factor):
        self.timestamp /= factor
        return self

    def __div__(self, factor):
        return (self /= factor)
'''

