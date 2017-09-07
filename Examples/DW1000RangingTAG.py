"""
This python script is used to configure the DW1000 chip as a tag for ranging functionalities. It must be used in conjunction with the RangingAnchor script. 
It requires the following modules: DW1000, DW1000Constants and monotonic.
"""


import DW1000
import monotonic
import DW1000Constants as C

LEN_DATA = 16
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



def millis():
    """
    This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
    is used to avoid having the chip stuck in an undesirable state.
    """
    return int(round(monotonic.monotonic()*C.MILLISECONDS))


def handleSent():
    """
    This is a callback called from the module's interrupt handler when a transmission was successful. 
    It sets the sentAck variable as True so the loop can continue.
    """        
    global sentAck
    sentAck = True


def handleReceived():
    """
    This is a callback called from the module's interrupt handler when a reception was successful. 
    It sets the received receivedAck as True so the loop can continue.
    """            
    global receivedAck
    receivedAck = True


def receiver():
    """
    This function configures the chip to prepare for a message reception.
    """    
    DW1000.newReceive()
    DW1000.receivePermanently()
    DW1000.startReceive()


def noteActivity():
    """
    This function records the time of the last activity so we can know if the device is inactive or not.
    """    
    global lastActivity
    lastActivity = millis()


def resetInactive():
    """
    This function restarts the default polling operation when the device is deemed inactive.
    """
    global expectedMsgId
    # print("Reset inactive")	
    expectedMsgId = C.POLL_ACK
    transmitPoll()
    noteActivity()


def transmitPoll():
    """
    This function sends the polling message which is the first transaction to enable ranging functionalities. 
    It checks if an anchor is operational.
    """    
    global data, lastPoll
    while (millis() - lastPoll < POLL_RANGE_FREQ):
        pass
    DW1000.newTransmit()
    data[0] = C.POLL
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()
    lastPoll = millis()


def transmitRange():
    """
    This function sends the range message containing the timestamps used to calculate the range between the devices.
    """
    global data, timeRangeSentTS
    DW1000.newTransmit()
    data[0] = C.RANGE
    timeRangeSentTS = DW1000.setDelay(REPLY_DELAY_TIME_US, C.MICROSECONDS)
    DW1000.setTimeStamp(data, timePollSentTS, 1)
    DW1000.setTimeStamp(data, timePollAckReceivedTS, 6)
    DW1000.setTimeStamp(data, timeRangeSentTS, 11)
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()


def loop():
    global sentAck, receivedAck, data, timePollAckReceivedTS, timePollSentTS, timeRangeSentTS, expectedMsgId
    if (sentAck == False and receivedAck == False):
        if ((millis() - lastActivity) > C.RESET_PERIOD):
            resetInactive()
        return

    if sentAck:
        sentAck = False
        msgID = data[0]      
        if msgID == C.POLL:
            timePollSentTS = DW1000.getTransmitTimestamp()
        elif msgID == C.RANGE:
            timeRangeSentTS = DW1000.getTransmitTimestamp()
            noteActivity()

    if receivedAck:
        receivedAck = False
        data = DW1000.getData(LEN_DATA)
        msgID = data[0]    
        if msgID != expectedMsgId:
            expectedMsgId = C.POLL_ACK
            transmitPoll()
            return
        if msgID == C.POLL_ACK:
            timePollAckReceivedTS = DW1000.getReceiveTimestamp()
            expectedMsgId = C.RANGE_REPORT
            transmitRange()
            noteActivity()
        elif msgID == C.RANGE_REPORT:
            expectedMsgId = C.POLL_ACK
            transmitPoll()
            noteActivity()
        elif msgID == C.RANGE_FAILED:
            expectedMsgId = C.POLL_ACK
            transmitPoll()
            noteActivity()


try:
    PIN_IRQ = 19
    PIN_SS = 16
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    print("DW1000 initialized")
    print("############### TAG ##############")	

    DW1000.generalConfiguration("7D:00:22:EA:82:60:3B:9C", C.MODE_LONGDATA_RANGE_ACCURACY)
    DW1000.registerCallback("handleSent", handleSent)
    DW1000.registerCallback("handleReceived", handleReceived)
    DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)

    receiver()
    transmitPoll()
    noteActivity()
    while 1:
        loop()

except KeyboardInterrupt:
    DW1000.close()
