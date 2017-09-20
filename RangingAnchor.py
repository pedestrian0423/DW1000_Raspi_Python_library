"""
This python script is used to configure the DW1000 chip as an anchor for ranging functionalities. It must be used in conjunction with the RangingTAG script. 
It requires the following modules: DW1000, DW1000Constants and monotonic.
"""


import DW1000
import monotonic
import DW1000Constants as C

class RangingAnchor(object):
    dw1000_device = None

    lastActivity = 0
    expectedMsgId = C.POLL
    protocolFailed = False
    sentAck = False
    receivedAck = False
    LEN_DATA = 18
    data = [0] * LEN_DATA
    timePollAckSentTS = 0
    timePollAckReceivedTS = 0
    timePollReceivedTS = 0
    timeRangeReceivedTS = 0
    timePollSentTS = 0
    timeRangeSentTS = 0
    timeComputedRangeTS = 0
    REPLY_DELAY_TIME_US = 7000 

    def __init__(self, **kwargs):
        self.dw1000_device = DW1000.DW1000(**kwargs)

    def millis(self):
        """
        This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
        is used to avoid having the chip stuck in an undesirable state.
        """    
        return int(round(monotonic.monotonic() * C.MILLISECONDS))


    def handleSent(self):
        """
        This is a callback called from the module's interrupt handler when a transmission was successful. 
        It sets the sentAck variable as True so the loop can continue.
        """            
        self.sentAck = True


    def handleReceived(self):
        """
        This is a callback called from the module's interrupt handler when a reception was successful. 
        It sets the received receivedAck as True so the loop can continue.
        """       
        self.receivedAck = True


    def noteActivity(self):
        """
        This function records the time of the last activity so we can know if the device is inactive or not.
        """        
        self.lastActivity = self.millis()


    def resetInactive(self):
        """
        This function restarts the default polling operation when the device is deemed inactive.
        """    
        # print("reset inactive")    
        self.expectedMsgId = C.POLL
        self.receiver()
        self.noteActivity()


    def transmitPollAck(self):
        """
        This function sends the polling acknowledge message which is used to confirm the reception of the polling message. 
        """        
        self.dw1000_device.newTransmit()
        for i in range(0, self.LEN_DATA):
            self.data[i] = 0
        self.data[0] = C.POLL_ACK
        self.data[1] = 0xEF
        self.data[2] = 0x01
        self.dw1000_device.setDelay(self.REPLY_DELAY_TIME_US, C.MICROSECONDS)
        self.dw1000_device.setData(self.data, self.LEN_DATA)
        self.dw1000_device.startTransmit()


    def transmitRangeAcknowledge(self):
        """
        This functions sends the range acknowledge message which tells the tag that the ranging function was successful and another ranging transmission can begin.
        """
        self.dw1000_device.newTransmit()
        for i in range(0, self.LEN_DATA):
            self.data[i] = 0
        self.data[0] = C.RANGE_REPORT
        self.data[1] = 0xEF
        self.data[2] = 0x01
        self.dw1000_device.setData(self.data, self.LEN_DATA)
        self.dw1000_device.startTransmit()


    def transmitRangeFailed(self):
        """
        This functions sends the range failed message which tells the tag that the ranging function has failed and to start another ranging transmission.
        """    
        self.dw1000_device.newTransmit()
        for i in range(0, self.LEN_DATA):
            self.data[i] = 0
        self.data[0] = C.RANGE_FAILED
        self.dw1000_device.setData(self.data, self.LEN_DATA)
        self.dw1000_device.startTransmit()


    def receiver(self):
        """
        This function configures the chip to prepare for a message reception.
        """    
        self.dw1000_device.newReceive()
        self.dw1000_device.receivePermanently()
        self.dw1000_device.startReceive()


    def computeRangeAsymmetric(self):
        """
        This is the function which calculates the timestamp used to determine the range between the devices.
        """
        round1 = self.dw1000_device.wrapTimestamp(self.timePollAckReceivedTS - self.timePollSentTS)
        reply1 = self.dw1000_device.wrapTimestamp(self.timePollAckSentTS - self.timePollReceivedTS)
        round2 = self.dw1000_device.wrapTimestamp(self.timeRangeReceivedTS - self.timePollAckSentTS)
        reply2 = self.dw1000_device.wrapTimestamp(self.timeRangeSentTS - self.timePollAckReceivedTS)
        self.timeComputedRangeTS = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)


    def loop(self):
        if (self.sentAck is False and self.receivedAck is False):
            if ((self.millis() - self.lastActivity) > C.RESET_PERIOD):
                self.resetInactive()
            return

        if self.sentAck:
            self.sentAck = False
            msgId = self.data[0]
            if msgId == C.POLL_ACK:
                self.timePollAckSentTS = self.dw1000_device.getTransmitTimestamp()
                self.noteActivity()

        if self.receivedAck:
            self.receivedAck = False
            self.data = self.dw1000_device.getData(self.LEN_DATA)
            msgId = self.data[0]
            if msgId != self.expectedMsgId:
                self.protocolFailed = True

            shortAddress = [0] * 2
            shortAddress[0] = self.data[1]
            shortAddress[1] = self.data[2]
            print("Short Address: %02X:%02X" % (shortAddress[1], shortAddress[0]))

            if msgId == C.POLL:
                self.protocolFailed = False
                self.timePollReceivedTS = self.dw1000_device.getReceiveTimestamp()
                self.expectedMsgId = C.RANGE
                self.transmitPollAck()
                self.noteActivity()
            elif msgId == C.RANGE:
                self.timeRangeReceivedTS = self.dw1000_device.getReceiveTimestamp()
                self.expectedMsgId = C.POLL
                if self.protocolFailed == False:
                    self.timePollSentTS = self.dw1000_device.getTimeStamp(self.data, 3)
                    self.timePollAckReceivedTS = self.dw1000_device.getTimeStamp(self.data, 8)
                    self.timeRangeSentTS = self.dw1000_device.getTimeStamp(self.data, 13)
                    self.computeRangeAsymmetric()
                    self.transmitRangeAcknowledge()
                    distance = (self.timeComputedRangeTS % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO
                    print("Distance: %.2f m" %(distance))

                else:
                    self.transmitRangeFailed()

                self.noteActivity()

irq = 5
ss = 6
rst = None
bus = 0
device = 0

rangingAnchor = RangingAnchor(irq=irq, rst=rst, bus=bus, device=device)

try:    
    #PIN_IRQ = 5
    #PIN_SS = 6
    rangingAnchor.dw1000_device.setup(ss)
    print("DW1000 initialized")
    print("############### ANCHOR ##############")

    rangingAnchor.dw1000_device.generalConfiguration("82:17:5B:D5:A9:9A:E2:9C", C.MODE_LONGDATA_RANGE_ACCURACY)
    rangingAnchor.dw1000_device.registerCallback("handleSent", rangingAnchor.handleSent)
    rangingAnchor.dw1000_device.registerCallback("handleReceived", rangingAnchor.handleReceived)
    rangingAnchor.dw1000_device.setAntennaDelay(C.ANTENNA_DELAY_RASPI)

    rangingAnchor.receiver()
    rangingAnchor.noteActivity()
    while 1:
        rangingAnchor.loop()

except KeyboardInterrupt:
    rangingAnchor.dw1000_device.close()
