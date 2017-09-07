"""
This python script is used to configure the DW1000 chip as a receiver and start receiving message permanently. It must be used in conjunction with the Sender script. 
It receives the message and then prints it in the console.
It requires the following modules: DW1000, DW1000Constants
"""


import DW1000
import DW1000Constants as C

received = False

def handleReceived():
    """
    This is a callback called from the module's interrupt handler when a reception was successful. 
    It sets the received variable as True so the loop can continue.
    """    
    global received
    received = True


def receiver():
    """
    This function configures the chip to prepare for a message reception.
    """    
    DW1000.newReceive()
    DW1000.receivePermanently()
    DW1000.startReceive()


try:
    PIN_IRQ = 19
    PIN_SS = 16 
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    print("DW1000 initialized")    

    DW1000.generalConfiguration("7D:00:22:EA:82:60:3B:9C", C.MODE_LONGDATA_RANGE_LOWPOWER)
    DW1000.registerCallback("handleReceived", handleReceived)
    receiver()
    while 1:
        if received:
            fpPwr = DW1000.getFirstPathPower()
            rxPwr = DW1000.getReceivePower()
            rcvQuality = DW1000.getReceiveQuality()
            msg = DW1000.getDataStr()
            print(msg)
            print("FP power: %f dBm" % (fpPwr))
            print("RX power: %f dBm" % (rxPwr))
            print("Signal quality: %f \n" % (rcvQuality))
            received = False
            
except KeyboardInterrupt:
    DW1000.close()
