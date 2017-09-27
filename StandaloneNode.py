#!/usr/bin/python

from SocketServer import TCPServer as TCP, StreamRequestHandler as SRH
from time import ctime

import DW1000
import monotonic
import DW1000Constants as C

class RangingRequestHandler(SRH):
    def handler(self):
        # doing ranging functionality
        remote_ip_addr = '100.000.000.000'
        print('Ranging request received from' + remote_ip_addr)

class StandaloneNode(object):
    def __init__(self, **kwargs):
        self.dw1000_device = DW1000(**kwargs)

irq = 5
ss = 6
rst = None
bus = 0
device = 0

HOST = ''
PORT = 21567
ADDR = (HOST, PORT)


standalone_node = DW1000(irq=irq, rst=rst, bus=bus, device=device)
standalone_node.dw1000_device.setup(ss)
print("DW1000 initialized")
print("############### ANCHOR ##############")

standalone_node.dw1000_device.generalConfiguration("82:17:5B:D5:A9:9A:E2:9C", C.MODE_LONGDATA_RANGE_ACCURACY)
standalone_node.dw1000_device.registerCallback("handleSent", standalone_node.handleSent)
standalone_node.dw1000_device.registerCallback("handleReceived", standalone_node.handleReceived)
standalone_node.dw1000_device.setAntennaDelay(C.ANTENNA_DELAY_RASPI)

standalone_node.receiver()
standalone_node.noteActivity()

tcpServ = TCP(ADDR, RangingRequestHandler)
print('Waiting for connection')
tcpServ.serve_forever()