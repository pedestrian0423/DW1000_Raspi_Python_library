"""
    This python module deal with medium access control (MAC) of DW1000
"""

import DW1000Constants as C
import DW1000Ranging

class DW1000Mac():
    _seq_number = 0

    def __init__(self):
        self._seq_number = 0

    def generate_blink_frame(self, source_address, source_short_address):
        """
            For poll message, authors use just 2 bytes address
            Totla: 12 bytes
        """
        # Initialize frame
        frame = bytearray()

        # Set constant and sequence number [0] and [1]
        frame.append(C.FC_1_BLINK)
        frame.append(self._seq_number)

        # Set reversed source address array [2 - 9]
        frame.append(source_address[7])
        frame.append(source_address[6])
        frame.append(source_address[5])
        frame.append(source_address[4])
        frame.append(source_address[3])
        frame.append(source_address[2])
        frame.append(source_address[1])
        frame.append(source_address[0])

        # Set reversed short address array [10 - 11]
        frame.append(source_short_address[1])
        frame.append(source_short_address[0])

        self.increment_seq_number()

        return frame

    def generate_short_mac_frame(self, source_short_address, destination_short_address):
        """
            The short frame usually for Resp, Final, or Report
            2 bytes for Destination Address and 2 bytes for Source Address
            Total: 9 bytes
        """

        # Init frame
        frame = bytearray()

        # Frame control
        frame.append(C.FC_1)
        frame.append(C.FC_2_SHORT)
        # Sequence number modulo 256
        frame.append(self._seq_number)

        # PAN ID
        frame.append(0xCA)
        frame.append(0xDE)

        # Set destination address
        frame.append(destination_short_address[1])
        frame.append(destination_short_address[0])

        # Set source address
        frame.append(source_short_address[1])
        frame.append(source_short_address[0])

        self.increment_seq_number()

        return frame

    def generate_long_mac_frame(self, source_short_address, destination_address):
        """
            The long frame for ranging init
            8 bytes for destination address and 2 bytes for source address
            Total: 15 bytes
        """
        
        # Initialize byte array
        frame = bytearray()

        frame.append(C.FC_1)    # 0
        frame.append(C.FC_2)    # 1

        frame.append(self._seq_number) # 2

        frame.append(0xCA)  # 3
        frame.append(0xDE)  # 4

        # Destination address (8 bytes, reversed)
        frame.append(destination_address[7])    # 5
        frame.append(destination_address[6])    # 6
        frame.append(destination_address[5])    # 7
        frame.append(destination_address[4])    # 8
        frame.append(destination_address[3])    # 9
        frame.append(destination_address[2])    # 10
        frame.append(destination_address[1])    # 11
        frame.append(destination_address[0])    # 12

        # Source address (2 bytes, reversed)
        frame.append(source_short_address[1])   # 13
        frame.append(source_short_address[0])   # 14

        self.increment_seq_number()

        return frame

    def decode_blink_frame(self, frame):
        reverse_address = bytearray()
        reverse_address.append(frame[9])
        reverse_address.append(frame[8])
        reverse_address.append(frame[7])
        reverse_address.append(frame[6])
        reverse_address.append(frame[5])
        reverse_address.append(frame[4])
        reverse_address.append(frame[3])
        reverse_address.append(frame[2])

        reverse_short_address = bytearray()
        reverse_short_address.append(frame[1])
        reverse_short_address.append(frame[0])

        return_list = [reverse_address, reverse_short_address]

        return return_list

    def decode_short_mac_frame(self, frame):
        address = bytearray()
        address.append(frame[1])
        address.append(frame[0])

        return address

    def decode_long_mac_frame(self, frame):
        address = bytearray()
        address.append(frame[14])
        address.append(frame[13])

        return address

    def increment_seq_number(self):
        if(self._seq_number == 255):
            self._seq_number = 0
        else:
            self._seq_number += 1
