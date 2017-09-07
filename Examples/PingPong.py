"""
This python script is used to configure the DW1000 chip as a device to emulate a ping/pong message transmission. 
It must be used in conjunction with another PingPong script setup as the opposite trxToggle, meaning that there should be two scripts : Receiver + Transmitter
It requires the following modules: DW1000, DW1000Constants
"""


import DW1000
import DW1000Constants as C

msg = ""
trxAck = False
# Change this value to set the chip as either a receiver or transmitter
trxToggle = C.RECEIVER

def handleSent():
    """
    This is a callback called from the module's interrupt handler when a transmission was successful. 
    It sets the sent trxAck as True so the loop can continue.
    """    
    global trxAck
    trxAck = True

def handleReceived():
    """
    This is a callback called from the module's interrupt handler when a reception was successful. 
    It sets the received trxAck as True so the loop can continue.
    """        
    global trxAck
    trxAck = True

def receiver():
    """
    This function configures the chip to prepare for a message reception.
    """
    DW1000.newReceive()
    DW1000.receivePermanently()
    DW1000.startReceive()

def transmitter():
    """
    This function configures the chip to prepare for a transmission.
    It modifies the data that will be sent and start the transmission with the chosen delay.
    """    
    DW1000.newTransmit()
    DW1000.setDataStr(msg)
    DW1000.setDelay(2000, C.MILLISECONDS)
    DW1000.startTransmit()

try:
    PIN_IRQ = 19
    PIN_SS = 16
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    print("DW1000 initialized ...")

    DW1000.generalConfiguration("FF:FF:FF:FF:00:00:00:00", C.MODE_LONGDATA_RANGE_LOWPOWER)
    DW1000.registerCallback("handleSent", handleSent)
    DW1000.registerCallback("handleReceived", handleReceived)
    if (trxToggle == C.TRANSMITTER):
        msg = "Ping...."
        receiver()
        transmitter()
    else:
        msg = "... and Pong"
        receiver()

    while 1:
        if trxAck:
            trxAck = False
            trxToggle = not trxToggle
            if trxToggle:
                rxMsg = DW1000.getDataStr()
                print("Received : " + rxMsg)
                transmitter()
            else:
                print("Transmitted: " + msg)


except KeyboardInterrupt:
    DW1000.close()    