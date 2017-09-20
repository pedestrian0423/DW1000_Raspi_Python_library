"""
This python script is used to configure the DW1000 chip as a tag for ranging functionalities. It must be used in conjunction with the RangingAnchor script. 
It requires the following modules: DW1000, DW1000Constants and monotonic.
"""


import DW1000
import monotonic
import DW1000Constants as C

class RangingTag():
    dw1000_device = None

    LEN_DATA = 18
    data = [0] * LEN_DATA
    lastActivity = 0
    lastPoll = 0
    sentAck = False
    receivedAck = False
    expectedMsgId = C.POLL_ACK
    timePollSentTS = 0
    timeRangeSentTS = 0
    timePollAckReceivedTS = 0
    REPLY_DELAY_TIME_US = 7000

    # The polling range frequency defines the time interval between every distance poll in milliseconds. Feel free to change its value. 
    POLL_RANGE_FREQ = 1000 # the distance between the tag and the anchor will be estimated every second.

    def __init__(self, **kwargs):
        self.dw1000_device = DW1000.DW1000(**kwargs)



    def millis(self):
        """
        This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
        is used to avoid having the chip stuck in an undesirable state.
        """
        return int(round(monotonic.monotonic()*C.MILLISECONDS))


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


    def receiver(self):
        """
        This function configures the chip to prepare for a message reception.
        """    
        self.dw1000_device.newReceive()
        self.dw1000_device.receivePermanently()
        self.dw1000_device.startReceive()


    def noteActivity(self):
        """
        This function records the time of the last activity so we can know if the device is inactive or not.
        """    
        self.lastActivity = self.millis()


    def resetInactive(self):
        """
        This function restarts the default polling operation when the device is deemed inactive.
        """
        # print("Reset inactive")	
        self.expectedMsgId = C.POLL_ACK
        self.transmitPoll()
        self.noteActivity()


    def transmitPoll(self):
        """
        This function sends the polling message which is the first transaction to enable ranging functionalities. 
        It checks if an anchor is operational.
        """    
        while (self.millis() - self.lastPoll < self.POLL_RANGE_FREQ):
            pass
        self.dw1000_device.newTransmit()
        for i in range(0, self.LEN_DATA):
            self.data[i] = 0
        self.data[0] = C.POLL
        self.data[1] = 0xAB
        self.data[2] = 0xCD
        self.dw1000_device.setData(self.data, self.LEN_DATA)
        self.dw1000_device.startTransmit()
        self.lastPoll = self.millis()


    def transmitRange(self):
        """
        This function sends the range message containing the timestamps used to calculate the range between the devices.
        """
        self.dw1000_device.newTransmit()
        for i in range(0, self.LEN_DATA):
            self.data[i] = 0
        self.data[0] = C.RANGE
        self.data[1] = 0xAB
        self.data[2] = 0xCD
        self.timeRangeSentTS = self.dw1000_device.setDelay(self.REPLY_DELAY_TIME_US, C.MICROSECONDS)
        self.dw1000_device.setTimeStamp(self.data, self.timePollSentTS, 3)
        self.dw1000_device.setTimeStamp(self.data, self.timePollAckReceivedTS, 8)
        self.dw1000_device.setTimeStamp(self.data, self.timeRangeSentTS, 13)
        self.dw1000_device.setData(self.data, self.LEN_DATA)
        self.dw1000_device.startTransmit()


    def loop(self):
        global sentAck, receivedAck, data, timePollAckReceivedTS, timePollSentTS, timeRangeSentTS, expectedMsgId
        if (self.sentAck == False and self.receivedAck == False):
            if ((self.millis() - self.lastActivity) > C.RESET_PERIOD):
                self.resetInactive()
            return

        if self.sentAck:
            self.sentAck = False
            msgID = self.data[0]      
            if msgID == C.POLL:
                self.timePollSentTS = self.dw1000_device.getTransmitTimestamp()
            elif msgID == C.RANGE:
                self.timeRangeSentTS = self.dw1000_device.getTransmitTimestamp()
                self.noteActivity()

        if self.receivedAck:
            self.receivedAck = False
            self.data = self.dw1000_device.getData(self.LEN_DATA)
            msgID = self.data[0]    
            if msgID != self.expectedMsgId:
                self.expectedMsgId = C.POLL_ACK
                self.transmitPoll()
                return
            
            shortAddress = [0] * 2
            shortAddress[0] = self.data[1]
            shortAddress[1] = self.data[2]
            print("Short Address: %02X:%02X" % (shortAddress[1], shortAddress[0]))

            if msgID == C.POLL_ACK:
                self.timePollAckReceivedTS = self.dw1000_device.getReceiveTimestamp()
                self.expectedMsgId = C.RANGE_REPORT
                self.transmitRange()
                self.noteActivity()
            elif msgID == C.RANGE_REPORT:
                self.expectedMsgId = C.POLL_ACK
                # need to add distance information to data
                self.transmitPoll()
                self.noteActivity()
            elif msgID == C.RANGE_FAILED:
                self.expectedMsgId = C.POLL_ACK
                self.transmitPoll()
                self.noteActivity()




irq = 5
ss = 6
rst = None
bus = 0
device = 0

rangingTag = RangingTag(irq=irq, rst=rst, bus=bus, device=device)

try:
    rangingTag.dw1000_device.setup(ss)
    print("DW1000 initialized")
    print("############### TAG ##############")	

    rangingTag.dw1000_device.generalConfiguration("7D:00:22:EA:82:60:3B:9C", C.MODE_LONGDATA_RANGE_ACCURACY)
    rangingTag.dw1000_device.registerCallback("handleSent", rangingTag.handleSent)
    rangingTag.dw1000_device.registerCallback("handleReceived", rangingTag.handleReceived)
    rangingTag.dw1000_device.setAntennaDelay(C.ANTENNA_DELAY_RASPI)

    rangingTag.receiver()
    rangingTag.transmitPoll()
    rangingTag.noteActivity()
    while 1:
        rangingTag.loop()

except KeyboardInterrupt:
    rangingTag.dw1000_device.close()
