"""
This python script is used to configure the DW1000 chip as a sender and start sending message . It must be used in conjunction with the Receiver script. 
It sends the custom message with a specified delay (2 sec by default).
It requires the following modules: DW1000, DW1000Constants
"""


import DW1000
import DW1000Constants as C

number = 1
sent = False
SEND_DELAY = 2000


def handleSent():
    """
    This is a callback called from the module's interrupt handler when a transmission was successful. 
    It sets the sent variable as True so the loop can continue.
    """
    global sent
    sent = True

def transmitter():
    """
    This function configures the chip to prepare for a transmission.
    It modifies the data that will be sent and start the transmission with the chosen delay.
    """
    global number
    DW1000.newTransmit()
    msg = "Hello Raspi3, it's #" + str(number)
    DW1000.setDataStr(msg)
    DW1000.setDelay(SEND_DELAY, C.MILLISECONDS)
    DW1000.startTransmit()
    number += 1

try:
    PIN_IRQ = 19
    PIN_SS = 16
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    print("DW1000 initialized ...")
    DW1000.generalConfiguration("7D:00:22:EA:82:60:3B:9C", C.MODE_LONGDATA_RANGE_LOWPOWER)    
    DW1000.registerCallback("handleSent", handleSent)
    transmitter()
    while 1:
        if sent:
            transmitter()
            sent=False

except KeyboardInterrupt:
    DW1000.close()
