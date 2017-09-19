"""
This python module contains low-level functions to interact
with the DW1000 chip using a Raspberry Pi 3.
It requires the following modules:
    math, time, spidev, Rpi.GPIO, random.
"""
import time
import math
from random import randint
import spidev
import RPi.GPIO as GPIO
import DW1000Constants as C

class DW1000(object):
    spi = spidev.SpiDev()
    _bus = 0
    _device = 0
    _ss = 0

    _chipSelect = None
    _deviceMode = C.IDLE_MODE
    _permanentReceive = False
    _operationMode = [None] * 6
        # [dataRate, pulseFrequency, pacSize, preambleLength, channel, preacode]
    callbacks = {}

    _networkAndAddress = [0] * 4
    _sysctrl = [0] * 4
    _chanctrl = [0] * 4
    _syscfg = [0] * 4
    _sysmask = [0] * 4
    _txfctrl = [0] * 5
    _sysstatus = [0] * 5

    """
    DW1000 general configuration.
    """
    def __init__(self, **kwargs):
        rst = None
        bus = None
        device = None

        valid_keys = ["irq", "rst", "bus", "device"]
        for key in valid_keys:
            self.__dict__[key] = kwargs.get(key)

        self.spi = spidev.SpiDev()
        
        self.begin(self.irq, self.rst, self.bus, self.device)


    def __del__(self):
        self.close()


    def begin(self, irq, rst=None, bus=None, device=None):
        """
        This function opens the SPI connection available on the Raspberry Pi using the chip select #0.
        Normally, spidev can auto enable chip select when necessary. 
        However, in our case, the dw1000's chip select is connected to GPIO16 so we have to enable/disable it manually.
        It also sets up the interrupt detection event on the rising edge of the interrupt pin.

        Args:
                irq : The GPIO pin number managing interrupts.
        """
        self._bus = 0
        self._device = 0
        self._rst = None
        self._irq = irq

        # Wait 5 us to open spi connection to let the chip enter idle state
        # see 2.3.2 of the DW1000 user manual (INIT).
        time.sleep(C.INIT_DELAY)
        GPIO.setmode(GPIO.BCM)

        if bus is not None:
            self._bus = bus
        if device is not None:
            self._device = device
        self.spi.open(self._bus, self._device)
        # spi.max_speed_hz = 4000000
        self._deviceMode = C.IDLE_MODE
        GPIO.setup(self._irq, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self._irq, GPIO.RISING, callback=self.handleInterrupt)

        # Set reset pin for physical reset
        if rst is not None:
            self._rst = rst
            GPIO.setup(self._rst, GPIO.INPUT)


    def setup(self, ss):
        """
        This function defines the GPIO used for the chip select
        by configuring it as an output and by setting its initial state at inactive (HIGH).
        It also clears interrupt configuration and performs
        a soft reset before applying initial configurations for the chip.

        Args:
                ss: The GPIO pin number of the chip enable/select for the SPI bus.
        """
        self._chipSelect = ss
        GPIO.setup(self._chipSelect, GPIO.OUT)
        GPIO.output(self._chipSelect, GPIO.HIGH)

        self.enableClock(C.AUTO_CLOCK)

        self.softReset()

        # Default system configuration
        self.setArray(self._syscfg, 4, 0x00)
        self.setBit(self._syscfg, 4, C.DIS_DRXB_BIT, True)
        self.setBit(self._syscfg, 4, C.HIRQ_POL_BIT, True)
        self.writeBytes(C.SYS_CFG, C.NO_SUB, self._syscfg, 4)

        # clear interrupts configuration
        self.setArray(self._sysmask, 4, 0x00)
        self.writeBytes(C.SYS_MASK, C.NO_SUB, self._sysmask, 4)

        self.enableClock(C.XTI_CLOCK)
        self.manageLDE()
        self.enableClock(C.AUTO_CLOCK)


    def handleInterrupt(self, channel):
        """
        Callback invoked on the rising edge of the interrupt pin. Handle the configured interruptions.
        """
        # print("\nInterrupt!")
        readBytes(C.SYS_STATUS, C.NO_SUB, self._sysstatus, 5)
        # print(_sysstatus)
        msgReceived = getBit(self._sysstatus, 5, C.RXFCG_BIT)
        receiveTimeStampAvailable = getBit(self._sysstatus, 5, C.LDEDONE_BIT)
        transmitDone = getBit(self._sysstatus, 5, C.TXFRS_BIT)
        if transmitDone:
            self.callbacks["handleSent"]()
            self.clearTransmitStatus()
        if receiveTimeStampAvailable:
            self.setBit(self._sysstatus, 5, C.LDEDONE_BIT, True)
            self.writeBytes(C.SYS_STATUS, C.NO_SUB, self._sysstatus, 5)
        if isReceiveFailed():
            self.clearReceiveStatus()
            if self._permanentReceive:
                self.newReceive()
                self.startReceive()
        elif isReceiveTimeout():
            self.clearReceiveStatus()
            if self._permanentReceive:
                self.newReceive()
                self.startReceive()
        elif msgReceived:
            self.callbacks["handleReceived"]()
            self.clearReceiveStatus()                
            if self._permanentReceive:
                # no need to start a new receive since we enabled the permanent receive mode in the system configuration register. it created an interference causing problem
                # with the reception
                # newReceive()
                self.startReceive()

        self.clearAllStatus()


    def registerCallback(self, string, callback):
        """
        This function saves the callback sent by the script who imports this module for later use. It stores it in a dictionary with the
        specified key.

        Args:
                string: This is the key used to store the callback in the dictionary.
                callback: This is the saved callback.
        """
        if callback not in self.callbacks:
            self.callbacks[string] = callback


    def softReset(self):
        """
        This function performs a soft reset on the DW1000 chip.
        """
        pmscctrl0 = [0] * 4
        self.readBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 4)
        pmscctrl0[0] = C.SOFT_RESET_SYSCLKS
        self.writeBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 4)
        pmscctrl0[3] = C.SOFT_RESET_CLEAR
        self.writeBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 4)
        pmscctrl0[0] = C.SOFT_RESET_CLEAR
        pmscctrl0[3] = C.SOFT_RESET_SET
        self.writeBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 4)
        self.idle()

    def hardReset(self):
        """
        This function performs a hard reset on the DW1000 chip.
        """
        GPIO.setup(self._rst, GPIO.output)
        GPIO.output(self._rst, GPIO.LOW)
        time.sleep(0.002)
        GPIO.setup(self._rst, GPIO.input)
        time.sleep(0.010)
        self.idle()


    def manageLDE(self):
        """
        This function manages the LDE micro-code. It is to setup the power management and system control unit as well as the OTP memory interface. 
        This is necessary as part of the DW1000 initialisation, since it is important to get timestamp and diagnostic info from received frames.
        """
        pmscctrl0 = [None] * 4
        otpctrl = [None] * 2
        self.readBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 4)
        self.readBytes(C.OTP_IF, C.OTP_CTRL_SUB, otpctrl, 2)

        pmscctrl0[0] = C.LDE_L1STEP1
        pmscctrl0[1] = C.LDE_L1STEP2
        otpctrl[0] = C.LDE_L2STEP1
        otpctrl[1] = C.LDE_L2STEP2

        self.writeBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 2)
        self.writeBytes(C.OTP_IF, C.OTP_CTRL_SUB, otpctrl, 2)

        # wait 150 us before writing the 0x36:00 sub-register, see 2.5.5.10 of the DW1000 user manual.
        time.sleep(C.PMSC_CONFIG_DELAY)

        pmscctrl0[0] = C.LDE_L3STEP1
        pmscctrl0[1] = C.LDE_L3STEP2

        self.writeBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 2)


    def setDefaultConfiguration(self):
        """
        This function sets the default mode on the chip initialization : MODE_LONGDATA_RANGE_LOWPOWER and with receive/transmit mask activated when in IDLE mode.
        """
        if self._deviceMode == C.TX_MODE:
            pass
        elif self._deviceMode == C.RX_MODE:
            pass
        elif self._deviceMode == C.IDLE_MODE:
            self._syscfg[2] &= C.ENABLE_MODE_MASK2
            self._syscfg[2] |= 0x00
            
            self.setBit(self._syscfg, 4, C.DIS_STXP_BIT, True)
            self.setBit(self._syscfg, 4, C.FFEN_BIT, False)
            
            # interrupt on sent
            self.setBit(self._sysmask, 4, C.MTXFRS_BIT, True)
            # interrupt on received
            self.setBit(self._sysmask, 4, C.MRXDFR_BIT, True)
            self.setBit(self._sysmask, 4, C.MRXFCG_BIT, True)
            # interrupt on received failed
            self.setBit(self._sysmask, 4, C.MLDEERR_BIT, True)
            self.setBit(self._sysmask, 4, C.MRXFCE_BIT, True)
            self.setBit(self._sysmask, 4, C.MRXPHE_BIT, True)
            self.setBit(self._sysmask, 4, C.MRXRFSL_BIT, True)
            # interrupt on receive time stamp available
            self.setBit(self._sysmask, 4, C.MLDEDONE_BIT, False)
            # interrupt on auto acknowledge trigger
            self.setBit(self._sysmask, 4, C.MAAT_BIT, True)
            # set receiver auto reenable
            self.setBit(self._syscfg, 4, C.RXAUTR_BIT,  True)

            self.clearAllStatus()

            self.enableMode(C.MODE_LONGDATA_RANGE_LOWPOWER)

    def enableMode(self, mode):
        """
        This function configures the DW1000 chip to perform with a specific mode. It sets up the TRX rate the TX pulse frequency and the preamble length.
        """

        # setDataRate
        rate = mode[0]
        rate = rate & C.MASK_LS_2BITS
        self._txfctrl[1] = self._txfctrl[1] & C.ENABLE_MODE_MASK1
        self._txfctrl[1] = self._txfctrl[1] | ((rate << 5) & C.MASK_LS_BYTE)
        if rate == C.TRX_RATE_110KBPS:
            self.setBit(self._syscfg, 4, C.RXM110K_BIT, True)
        else:
            self.setBit(self._syscfg, 4, C.RXM110K_BIT, False)

        if rate == C.TRX_RATE_6800KBPS:
            self.setBit(self._chanctrl, 4, C.DWSFD_BIT, False)
            self.setBit(self._chanctrl, 4, C.TNSSFD_BIT, False)
            self.setBit(self._chanctrl, 4, C.RNSSFD_BIT, False)
        else:
            self.setBit(self._chanctrl, 4, C.DWSFD_BIT, True)
            self.setBit(self._chanctrl, 4, C.TNSSFD_BIT, True)
            self.setBit(self._chanctrl, 4, C.RNSSFD_BIT, True)
        if rate == C.TRX_RATE_850KBPS:
            sfdLength = [C.SFD_LENGTH_850KBPS]
        elif rate == C.TRX_RATE_6800KBPS:
            sfdLength = [C.SFD_LENGTH_6800KBPS]
        else:
            sfdLength = [C.SFD_LENGTH_OTHER]
        self.writeBytes(C.USR_SFD, C.SFD_LENGTH_SUB, sfdLength, 1)
        self._operationMode[C.DATA_RATE_BIT] = rate

        # setPulseFreq
        freq = mode[1]
        freq = freq & C.MASK_LS_2BITS
        self._txfctrl[2] = self._txfctrl[2] & C.ENABLE_MODE_MASK2
        self._txfctrl[2] = self._txfctrl[2] | (freq & C.MASK_LS_BYTE)
        self._chanctrl[2] = self._chanctrl[2] & C.ENABLE_MODE_MASK3
        self._chanctrl[2] = self._chanctrl[2] | ((freq << 2) & C.MASK_LS_BYTE)
        self._operationMode[C.PULSE_FREQUENCY_BIT] = freq

        # setPreambleLength
        prealen = mode[2]
        prealen = prealen & C.MASK_NIBBLE
        self._txfctrl[2] = self._txfctrl[2] & C.ENABLE_MODE_MASK4
        self._txfctrl[2] = self._txfctrl[2] | ((prealen << 2) & C.MASK_LS_BYTE)
        if prealen == C.TX_PREAMBLE_LEN_64 or prealen == C.TX_PREAMBLE_LEN_128:
            self._operationMode[C.PAC_SIZE_BIT] = C.PAC_SIZE_8
        elif prealen == C.TX_PREAMBLE_LEN_256 or prealen == C.TX_PREAMBLE_LEN_512:
            self._operationMode[C.PAC_SIZE_BIT] = C.PAC_SIZE_16
        elif prealen == C.TX_PREAMBLE_LEN_1024:
            self._operationMode[C.PAC_SIZE_BIT] = C.PAC_SIZE_32
        else:
            self._operationMode[C.PAC_SIZE_BIT] = C.PAC_SIZE_64
        self._operationMode[C.PREAMBLE_LENGTH_BIT] = prealen

        # setChannel
        self.setChannel(C.CHANNEL_5)

        # setPreambleCode
        if mode[1] == C.TX_PULSE_FREQ_16MHZ:
            self.setPreambleCode(C.PREAMBLE_CODE_16MHZ_4)
        else:
            self.setPreambleCode(C.PREAMBLE_CODE_64MHZ_10)

    def newConfiguration(self):
        """
        This function resets the DW1000 chip to the idle state mode and reads all the configuration registers to prepare for a new configuration.
        """
        self.idle()
        self.readBytes(C.PANADR, C.NO_SUB, self._networkAndAddress, 4)
        self.readBytes(C.SYS_CFG, C.NO_SUB, self._syscfg, 4)
        self.readBytes(C.CHAN_CTRL, C.NO_SUB, self._chanctrl, 4)
        self.readBytes(C.TX_FCTRL, C.NO_SUB, self._txfctrl, 5)
        self.readBytes(C.SYS_MASK, C.NO_SUB, self._sysmask, 4)

    def commitConfiguration(self):
        """
        This function commits the configuration stored in the arrays previously filled. It writes into the corresponding registers to apply the changes to the DW1000 chip.
        It also tunes the chip according to the current enabled mode.
        """
        self.writeBytes(C.PANADR, C.NO_SUB, self._networkAndAddress, 4)
        self.writeBytes(C.SYS_CFG, C.NO_SUB, self._syscfg, 4)
        self.writeBytes(C.CHAN_CTRL, C.NO_SUB, self._chanctrl, 4)
        self.writeBytes(C.TX_FCTRL, C.NO_SUB, self._txfctrl, 5)
        self.writeBytes(C.SYS_MASK, C.NO_SUB, self._sysmask, 4)

        self.tune()


    def setAntennaDelay(self, val):
        """
        This function sets the DW1000 chip's antenna delay value which needs to be calibrated to have better ranging accuracy.

        Args:
                val : The antenna delay value which will be configured into the chip.
        """
        antennaDelayBytes = [None] * 5
        self.writeValueToBytes(antennaDelayBytes, val, 5)
        self.writeBytes(C.TX_ANTD, C.NO_SUB, antennaDelayBytes, 2)
        self.writeBytes(C.LDE_CTRL, C.LDE_RXANTD_SUB, antennaDelayBytes, 2)


    def setEUI(self, currentAddress):
        """
        This function sets the extended unique identifier of the chip according to the value specified by the user in setup.

        Args:
                currentAddress : the array of bytes containing the EUI
        """
        reverseEUI = [0] * 8
        for i in range(0, 8):
            reverseEUI[i] = currentAddress[8 - i - 1]
        self.writeBytes(C.EUI, C.NO_SUB, reverseEUI, 8)


    def setDeviceAddress(self, value):
        """
        This function sets the device's address according to the specified value.

        Args:
                value : The address you want to set to the chip.
        """
        self._networkAndAddress[0] = value & C.MASK_LS_BYTE
        self._networkAndAddress[1] = (value >> 8) & C.MASK_LS_BYTE


    def setNetworkId(self, value):
        """
        This function sets the device's network ID according to the specified value.

        Args:
                value : The network id you want to assign to the chip.
        """
        self._networkAndAddress[2] = value & C.MASK_LS_BYTE
        self._networkAndAddress[3] = (value >> 8) & C.MASK_LS_BYTE


    def setChannel(self, channel):
        """
        This function configures the DW1000 chip to enable a the specified channel of operation.

        Args:
                channel : The channel value you want to assign to the chip.
        """
        channel = channel & C.MASK_NIBBLE
        self._chanctrl[0] = ((channel | (channel << 4)) & C.MASK_LS_BYTE)
        self._operationMode[C.CHANNEL_BIT] = channel


    def setPreambleCode(self, preacode):
        """
        This function sets the preamble code used for the frames, depending on the the pulse repetition frequency and the channel used.

        Args:
                preacode : The preamble code type you want to assign to the chip.
        """
        preacode = preacode & C.PREACODE_MASK1
        self._chanctrl[2] = self._chanctrl[2] & C.PREACODE_MASK2
        self._chanctrl[2] = self._chanctrl[2] | ((preacode << 6) & C.MASK_LS_BYTE)
        self._chanctrl[3] = 0x00
        self._chanctrl[3] = ((((preacode >> 2) & C.PREACODE_MASK3) |
                        (preacode << 3)) & C.MASK_LS_BYTE)
        self._operationMode[C.PREAMBLE_CODE_BIT] = preacode


    def tune(self):
        """
        This function tunes/configures dw1000 chip's registers according to the enabled mode. Although the DW1000 will power up in a usable mode for the default configuration,
        some of the register defaults are sub optimal and should be overwritten before using the chip in the default mode. See 2.5.5 of the user manual.
        """
        agctune1 = [None] * 2
        agctune2 = [None] * 4
        agctune3 = [None] * 2
        drxtune0b = [None] * 2
        drxtune1a = [None] * 2
        drxtune1b = [None] * 2
        drxtune2 = [None] * 4
        drxtune4H = [None] * 2
        rfrxctrlh = [None] * 1
        rftxctrl = [None] * 4
        tcpgdelay = [None] * 1
        fspllcfg = [None] * 4
        fsplltune = [None] * 1
        ldecfg1 = [None] * 1
        ldecfg2 = [None] * 2
        lderepc = [None] * 2
        txpower = [None] * 4
        fsxtalt = [None] * 1
        preambleLength = self._operationMode[C.PREAMBLE_LENGTH_BIT]
        channel = self._operationMode[C.CHANNEL_BIT]

        self.tuneAgcTune1(agctune1)
        self.writeValueToBytes(agctune2, C.AGC_TUNE2_OP, 4)
        self.writeValueToBytes(agctune3, C.AGC_TUNE3_OP, 2)
        self.tuneDrxTune0b(drxtune0b)
        self.tuneDrxTune1aAndldecfg2(drxtune1a, ldecfg2)
        self.tuneDrxtune1b(drxtune1b)
        self.tuneDrxTune2(drxtune2)

        if preambleLength == C.TX_PREAMBLE_LEN_64:
            self.writeValueToBytes(drxtune4H, C.DRX_TUNE4H_64, 2)
        else:
            self.writeValueToBytes(drxtune4H, C.DRX_TUNE4H_128, 2)

        if (channel != C.CHANNEL_4 and channel != C.CHANNEL_7):
            self.writeValueToBytes(rfrxctrlh, C.RF_RXCTRLH_1235, 1)
        else:
            self.writeValueToBytes(rfrxctrlh, C.RF_RXCTRLH_147, 1)

        self.tuneAccToChan(rftxctrl, tcpgdelay, fspllcfg, fsplltune, txpower)

        self.writeValueToBytes(ldecfg1, C.LDE_CFG1_OP, 1)
        self.tunelderepc(lderepc)

        buf_otp = [None] * 4
        buf_otp = self.readBytesOTP(C.OTP_XTAL_ADDRESS, buf_otp)
        if buf_otp[0] == 0:
            self.writeValueToBytes(
                fsxtalt, ((C.TUNE_OPERATION & C.TUNE_MASK1) | C.TUNE_MASK2), 1)
        else:
            self.writeValueToBytes(
                fsxtalt, ((buf_otp[0] & C.TUNE_MASK1) | C.TUNE_MASK2), 1)

        self.writeBytes(C.AGC_CTRL, C.AGC_TUNE1_SUB, agctune1, 2)
        self.writeBytes(C.AGC_CTRL, C.AGC_TUNE2_SUB, agctune2, 4)
        self.writeBytes(C.AGC_CTRL, C.AGC_TUNE3_SUB, agctune3, 2)
        self.writeBytes(C.DRX_CONF, C.DRX_TUNE0b_SUB, drxtune0b, 2)
        self.writeBytes(C.DRX_CONF, C.DRX_TUNE1a_SUB, drxtune1a, 2)
        self.writeBytes(C.DRX_CONF, C.DRX_TUNE1b_SUB, drxtune1b, 2)
        self.writeBytes(C.DRX_CONF, C.DRX_TUNE2_SUB, drxtune2, 4)
        self.writeBytes(C.DRX_CONF, C.DRX_TUNE4H_SUB, drxtune4H, 2)
        self.writeBytes(C.LDE_CTRL, C.LDE_CFG1_SUB, ldecfg1, 1)
        self.writeBytes(C.LDE_CTRL, C.LDE_CFG2_SUB, ldecfg2, 2)
        self.writeBytes(C.LDE_CTRL, C.LDE_REPC_SUB, lderepc, 2)
        self.writeBytes(C.TX_POWER, C.NO_SUB, txpower, 4)
        self.writeBytes(C.RF_CONF, C.RF_RXCTRLH_SUB, rfrxctrlh, 1)
        self.writeBytes(C.RF_CONF, C.RF_TXCTRL_SUB, rftxctrl, 4)
        self.writeBytes(C.TX_CAL, C.TC_PGDELAY_SUB, tcpgdelay, 1)
        self.writeBytes(C.FS_CTRL, C.FS_PLLTUNE_SUB, fsplltune, 1)
        self.writeBytes(C.FS_CTRL, C.FS_PLLCFG_SUB, fspllcfg, 4)
        self.writeBytes(C.FS_CTRL, C.FS_XTALT_SUB, fsxtalt, 1)


    def generalConfiguration(self, address, mode):
        """
        This function configures the DW1000 chip with general settings. It also defines the address and the network ID used by the device. It finally prints the
        configured device.

        Args:
                address: The string address you want to set the device to.
        """
        currentAddress = self.convertStringToByte(address)
        currentShortAddress = [0] * 2
        self.setEUI(currentAddress)
        currentShortAddress[0] = randint(0, 256)
        currentShortAddress[1] = randint(0, 256)
        deviceAddress = currentShortAddress[0] * 256 + currentShortAddress[1]

        # configure mode, network
        self.newConfiguration()
        self.setDefaultConfiguration()
        # setDeviceAddress(2)
        self.setDeviceAddress(deviceAddress)
        # setNetworkId(10)
        self.setNetworkId(0xDECA)
        self.enableMode(mode)
        self.setAntennaDelay(C.ANTENNA_DELAY)
        self.commitConfiguration()

        data = [0] * 4
        data2 = [0] * 8
        data3 = [0] * 4
        self.readBytes(C.DEV_ID, C.NO_SUB, data, 4)
        self.readBytes(C.EUI, C.NO_SUB, data2, 8)
        self.readBytes(C.PANADR, C.NO_SUB, data3, 4)
        print("\nDevice ID %02X - model: %d, version: %d, revision: %d" %
            ((data[3] << 8) | data[2], (data[1]), (data[0] >> 4) & C.MASK_NIBBLE, data[0] & C.MASK_NIBBLE))
        print("Unique ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X" % (
            data2[7], data2[6], data2[5], data2[4], data2[3], data2[2], data2[1], data2[0]))
        print("Network ID & Device Address: PAN: %02X, Short Address: %02X" %
            (((data3[3] << 8) | data3[2]), ((data3[1] << 8) | data3[0])))
        self.getDeviceModeInfo()


    """
    Tuning functions
    See tune() for more information
    """


    def tuneAgcTune1(self, data):
        """
        This function fills the array for the tuning of agctune1 according to the datasheet and the enabled mode.

        Args:
                data: The array which will store the correct values for agctune1.

        """
        pulseFrequency = self._operationMode[C.PULSE_FREQUENCY_BIT]
        if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
            self.writeValueToBytes(data, C.AGC_TUNE1_16MHZ_OP, 2)
        elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
            self.writeValueToBytes(data, C.AGC_TUNE1_DEFAULT_OP, 2)


    def tuneDrxTune0b(self, data):
        """
        This function fills the array for the tuning of drxtune0b according to the datasheet and the enabled mode.

        Args:
                data: The array which will store the correct values for drxtune0b.
        """
        dataRate = self._operationMode[C.DATA_RATE_BIT]
        if dataRate == C.TRX_RATE_110KBPS:
            self.writeValueToBytes(data, C.DRX_TUNE0b_110KBPS_NOSTD_OP, 2)
        elif dataRate == C.TRX_RATE_850KBPS:
            self.writeValueToBytes(data, C.DRX_TUNE0b_850KBPS_NOSTD_OP, 2)
        elif dataRate == C.TRX_RATE_6800KBPS:
            self.writeValueToBytes(data, C.DRX_TUNE0b_6800KBPS_STD_OP, 2)


    def tuneDrxTune1aAndldecfg2(self, data, data2):
        """
        This function fills the array for the tuning of drxtune1a and ldecfg2 according to the datasheet and the enabled mode.

        Args:
                data: The array which will store the correct values for the drxtune1a.    
                data2: The array which will store the correct values for ldecfg2.    
        """
        pulseFrequency = self._operationMode[C.PULSE_FREQUENCY_BIT]
        if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
            self.writeValueToBytes(data, C.DRX_TUNE1a_16MHZ_OP, 2)
            self.writeValueToBytes(data2, C.LDE_CFG2_16MHZ_OP, 2)
        elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
            self.writeValueToBytes(data, C.DRX_TUNE1a_64MHZ_OP, 2)
            self.writeValueToBytes(data2, C.LDE_CFG2_64MHZ_OP, 2)


    def tuneDrxtune1b(self, data):
        """
        This function fills the array for the tuning of drxtune1b according to the datasheet and the enabled mode.

        Args:
                data: The array which will store the correct values for drxtune1b.    
        """
        dataRate = self._operationMode[C.DATA_RATE_BIT]
        preambleLength = self._operationMode[C.PREAMBLE_LENGTH_BIT]
        if (preambleLength == C.TX_PREAMBLE_LEN_1536 or preambleLength == C.TX_PREAMBLE_LEN_2048 or preambleLength == C.TX_PREAMBLE_LEN_4096):
            if (dataRate == C.TRX_RATE_110KBPS):
                self.writeValueToBytes(data, C.DRX_TUNE1b_M1024, 2)
        elif preambleLength != C.TX_PREAMBLE_LEN_64:
            if (dataRate == C.TRX_RATE_850KBPS or dataRate == C.TRX_RATE_6800KBPS):
                self.writeValueToBytes(data, C.DRX_TUNE1b_L1024, 2)
        else:
            if dataRate == C.TRX_RATE_6800KBPS:
                self.writeValueToBytes(data, C.DRX_TUNE1b_64, 2)


    def tuneDrxTune2(self, data):
        """
        This function fills the array for the tuning of drxtune2 according to the datasheet and the enabled mode.

        Args:
                data: The array which will store the correct values for drxtune2.
        """
        pacSize = self._operationMode[C.PAC_SIZE_BIT]
        pulseFrequency = self._operationMode[C.PULSE_FREQUENCY_BIT]
        if pacSize == C.PAC_SIZE_8:
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_8_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_8_64MHZ, 4)
        elif pacSize == C.PAC_SIZE_16:
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_16_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_16_64MHZ, 4)
        elif pacSize == C.PAC_SIZE_32:
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_32_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_32_64MHZ, 4)
        elif pacSize == C.PAC_SIZE_64:
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_64_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(data, C.DRX_TUNE2_64_64MHZ, 4)


    def tuneAccToChan(self, rftxctrl, tcpgdelay, fspllcfg, fsplltune, txpower):
        """
        This function fills the arrays for the tuning of rftxctrl, tcpgdelay, fspllcfg, fsplltune and txpower according to the datasheet and the enabled mode.
        """
        channel = self._operationMode[C.CHANNEL_BIT]
        pulseFrequency = self._operationMode[C.PULSE_FREQUENCY_BIT]
        if channel == C.CHANNEL_1:
            self.writeValueToBytes(rftxctrl, C.RF_TXCTRL_1, 4)
            self.writeValueToBytes(tcpgdelay, C.TC_PGDELAY_1, 1)
            self.writeValueToBytes(fspllcfg, C.FS_PLLCFG_1, 4)
            self.writeValueToBytes(fsplltune, C.FS_PLLTUNE_1, 1)
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_12_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_12_64MHZ, 4)
        elif channel == C.CHANNEL_2:
            self.writeValueToBytes(rftxctrl, C.RF_TXCTRL_2, 4)
            self.writeValueToBytes(tcpgdelay, C.TC_PGDELAY_2, 1)
            self.writeValueToBytes(fspllcfg, C.FS_PLLCFG_24, 4)
            self.writeValueToBytes(fsplltune, C.FS_PLLTUNE_24, 1)
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_12_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_12_64MHZ, 4)
        elif channel == C.CHANNEL_3:
            self.writeValueToBytes(rftxctrl, C.RF_TXCTRL_3, 4)
            self.writeValueToBytes(tcpgdelay, C.TC_PGDELAY_3, 1)
            self.writeValueToBytes(fspllcfg, C.FS_PLLCFG_3, 4)
            self.writeValueToBytes(fsplltune, C.FS_PLLTUNE_3, 1)
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_3_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_3_64MHZ, 4)
        elif channel == C.CHANNEL_4:
            self.writeValueToBytes(rftxctrl, C.RF_TXCTRL_4, 4)
            self.writeValueToBytes(tcpgdelay, C.TC_PGDELAY_4, 1)
            self.writeValueToBytes(fspllcfg, C.FS_PLLCFG_24, 4)
            self.writeValueToBytes(fsplltune, C.FS_PLLTUNE_24, 1)
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_4_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_4_64MHZ, 4)
        elif channel == C.CHANNEL_5:
            self.writeValueToBytes(rftxctrl, C.RF_TXCTRL_5, 4)
            self.writeValueToBytes(tcpgdelay, C.TC_PGDELAY_5, 1)
            self.writeValueToBytes(fspllcfg, C.FS_PLLCFG_57, 4)
            self.writeValueToBytes(fsplltune, C.FS_PLLTUNE_57, 1)
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_5_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_5_64MHZ, 4)
        elif channel == C.CHANNEL_7:
            self.writeValueToBytes(rftxctrl, C.RF_TXCTRL_7, 4)
            self.writeValueToBytes(tcpgdelay, C.TC_PGDELAY_7, 1)
            self.writeValueToBytes(fspllcfg, C.FS_PLLCFG_57, 4)
            self.writeValueToBytes(fsplltune, C.FS_PLLTUNE_57, 1)
            if pulseFrequency == C.TX_PULSE_FREQ_16MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_7_16MHZ, 4)
            elif pulseFrequency == C.TX_PULSE_FREQ_64MHZ:
                self.writeValueToBytes(txpower, C.TX_POWER_7_64MHZ, 4)


    def tunelderepc(self, data):
        """
        This function fills the arrays for the tuning of lderepc according to the datasheet and the enabled mode.

        Args:
                data: The array which will store the correct values for lderepC.
        """
        preacode = self._operationMode[C.PREAMBLE_CODE_BIT]
        dataRate = self._operationMode[C.DATA_RATE_BIT]
        if (preacode == C.PREAMBLE_CODE_16MHZ_1 or preacode == C.PREAMBLE_CODE_16MHZ_2):
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_1AND2 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_1AND2, 2)
        elif (preacode == C.PREAMBLE_CODE_16MHZ_3 or preacode == C.PREAMBLE_CODE_16MHZ_8):
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_3AND8 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_3, 2)
        elif preacode == C.PREAMBLE_CODE_16MHZ_4:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_4 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_4, 2)
        elif preacode == C.PREAMBLE_CODE_16MHZ_5:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_5 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_5, 2)
        elif preacode == C.PREAMBLE_CODE_16MHZ_6:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_6 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_6, 2)
        elif preacode == C.PREAMBLE_CODE_16MHZ_7:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_7 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_7, 2)
        elif preacode == C.PREAMBLE_CODE_64MHZ_9:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_9 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_9, 2)
        elif preacode == C.PREAMBLE_CODE_64MHZ_10 or preacode == C.PREAMBLE_CODE_64MHZ_17:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_1017 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_1017, 2)
        elif preacode == C.PREAMBLE_CODE_64MHZ_11:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_111321 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_111321, 2)
        elif preacode == C.PREAMBLE_CODE_64MHZ_12:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_12 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_12, 2)
        elif preacode == C.PREAMBLE_CODE_64MHZ_18 or preacode == C.PREAMBLE_CODE_64MHZ_19:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_14161819 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_14161819, 2)
        elif preacode == C.PREAMBLE_CODE_64MHZ_20:
            if dataRate == C.TRX_RATE_110KBPS:
                self.writeValueToBytes(
                    data, ((C.LDE_REPC_20 >> 3) & C.MASK_LS_2BYTES), 2)
            else:
                self.writeValueToBytes(data, C.LDE_REPC_20, 2)


    def enableClock(self, clock):
        """
        This function manages the dw1000 chip's clock by setting up the proper registers to activate the specified clock mode chosen.

        Args:
                clock: An hex value corresponding to the clock mode wanted:
                    AUTO=0x00
                    XTI=0x01
                    PLL=0X02.
        """
        pmscctrl0 = [None] * 4
        self.readBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 4)
        if clock == C.AUTO_CLOCK:
            pmscctrl0[0] = C.AUTO_CLOCK
            pmscctrl0[1] = pmscctrl0[1] & C.ENABLE_CLOCK_MASK1
        elif clock == C.XTI_CLOCK:
            pmscctrl0[0] = pmscctrl0[0] & C.ENABLE_CLOCK_MASK2
            pmscctrl0[0] = pmscctrl0[0] | 1
        self.writeBytes(C.PMSC, C.PMSC_CTRL0_SUB, pmscctrl0, 2)


    def idle(self):
        """
        This function puts the chip into idle mode.
        """
        self.setArray(self._sysctrl, 4, 0x00)
        self.setBit(self._sysctrl, 4, C.TRXOFF_BIT, True)
        self._deviceMode = C.IDLE_MODE
        self.writeBytes(C.SYS_CTRL, C.NO_SUB, self._sysctrl, 4)


    """
    Message reception functions.
    """


    def newReceive(self):
        """
        This function prepares the chip for a new reception. It clears the system control register and also clear the RX latched bits in the SYS_STATUS register.
        """
        self.idle()
        self.setArray(self._sysctrl, 4, 0x00)
        self.clearReceiveStatus()
        self._deviceMode = C.RX_MODE


    def startReceive(self):
        """
        This function configures the chip to start the reception of a message sent by another DW1000 chip. 
        It turns on its receiver by setting RXENAB in the system control register.
        """
        self.setBit(self._sysctrl, 4, C.SFCST_BIT, False)
        self.setBit(self._sysctrl, 4, C.RXENAB_BIT, True)
        self.writeBytes(C.SYS_CTRL, C.NO_SUB, self._sysctrl, 4)


    def receivePermanently(self):
        """
        This function configures the dw1000 chip to receive data permanently. 
        """
        self._permanentReceive = True
        self.setBit(self._syscfg, 4, C.RXAUTR_BIT, True)
        self.writeBytes(C.SYS_CFG, C.NO_SUB, self._syscfg, 4)


    def isReceiveFailed(self):
        """
        This function reads the system event status register and checks if the message reception failed.

        Returns:
                True if the reception failed.
                False otherwise.
        """
        ldeErr = self.getBit(self._sysstatus, 5, C.LDEERR_BIT)
        rxCRCErr = self.getBit(self._sysstatus, 5, C.RXFCE_BIT)
        rxHeaderErr = self.getBit(self._sysstatus, 5, C.RXPHE_BIT)
        rxDecodeErr = self.getBit(self._sysstatus, 5, C.RXRFSL_BIT)
        if ldeErr or rxCRCErr or rxHeaderErr or rxDecodeErr:
            return True
        else:
            return False


    def isReceiveTimeout(self):
        """
        This function reads the system event status register and checks if there was a timeout in the message reception.

        Returns:
                True if there was a timeout in the reception.
                False otherwise.
        """
        isTimeout = self.getBit(self._sysstatus, 5, C.RXRFTO_BIT) | self.getBit(
            self._sysstatus, 5, C.RXPTO_BIT) | self.getBit(self._sysstatus, 5, C.RXSFDTO_BIT)
        return isTimeout


    def clearReceiveStatus(self):
        """
        This function clears the system event status register at the bits related to the reception of a message.
        """
        self.setBit(self._sysstatus, 5, C.RXDFR_BIT, True)
        self.setBit(self._sysstatus, 5, C.LDEDONE_BIT, True)
        self.setBit(self._sysstatus, 5, C.LDEERR_BIT, True)
        self.setBit(self._sysstatus, 5, C.RXPHE_BIT, True)
        self.setBit(self._sysstatus, 5, C.RXFCE_BIT, True)
        self.setBit(self._sysstatus, 5, C.RXFCG_BIT, True)
        self.setBit(self._sysstatus, 5, C.RXRFSL_BIT, True)

        self.writeBytes(C.SYS_STATUS, C.NO_SUB, self._sysstatus, 5)


    def getFirstPathPower(self):
        """
        This function calculates an estimate of the power in the first path signal. See section 4.7.1 of the DW1000 user manual for further details on the calculations.

        Returns:
                The estimated power in the first path signal.
        """
        fpAmpl1Bytes = [None] * 2
        fpAmpl2Bytes = [None] * 2
        fpAmpl3Bytes = [None] * 2
        rxFrameInfo = [None] * 4
        self.readBytes(C.RX_TIME, C.FP_AMPL1_SUB, fpAmpl1Bytes, 2)
        self.readBytes(C.RX_FQUAL, C.FP_AMPL2_SUB, fpAmpl2Bytes, 2)
        self.readBytes(C.RX_FQUAL, C.FP_AMPL3_SUB, fpAmpl3Bytes, 2)
        self.readBytes(C.RX_FINFO, C.NO_SUB, rxFrameInfo, 4)
        f1 = fpAmpl1Bytes[0] | fpAmpl1Bytes[1] << 8
        f2 = fpAmpl2Bytes[0] | fpAmpl2Bytes[1] << 8
        f3 = fpAmpl3Bytes[0] | fpAmpl3Bytes[1] << 8
        N = ((rxFrameInfo[2] >> 4) & C.MASK_LS_BYTE) | (rxFrameInfo[3] << 4)
        if self._operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_16MHZ:
            A = C.A_16MHZ
            corrFac = C.CORRFAC_16MHZ
        else:
            A = C.A_64MHZ
            corrFac = C.CORRFAC_64MHZ
        estFPPower = C.PWR_COEFF2 * \
            math.log10((f1 * f1 + f2 * f2 + f3 * f3) / (N * N)) - A
        if estFPPower <= -C.PWR_COEFF:
            return estFPPower
        else:
            estFPPower += (estFPPower + C.PWR_COEFF) * corrFac
        return estFPPower

    def getReceivePower(self):
        """
        This function calculates an estimate of the receive power level. See section 4.7.2 of the DW1000 user manual for further details on the calculation.

        Returns:
                The estimated receive power for the current reception.
        """
        cirPwrBytes = [None] * 2
        rxFrameInfo = [None] * 4
        readBytes(C.RX_FQUAL, C.CIR_PWR_SUB, cirPwrBytes, 2)
        readBytes(C.RX_FINFO, C.NO_SUB, rxFrameInfo, 4)
        cir = cirPwrBytes[0] | cirPwrBytes[1] << 8
        N = ((rxFrameInfo[2] >> 4) & C.MASK_LS_BYTE) | rxFrameInfo[3] << 4
        if _operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_16MHZ:
            A = C.A_16MHZ
            corrFac = C.CORRFAC_16MHZ
        else:
            A = C.A_64MHZ
            corrFac = C.CORRFAC_64MHZ
        estRXPower = 0
        if (float(cir) * float(C.TWOPOWER17)) / (float(N) * float(N)) > 0:
            estRXPower = C.PWR_COEFF2 * math.log10((float(cir) * float(C.TWOPOWER17)) / (float(N) * float(N))) - A
        if estRXPower <= -C.PWR_COEFF:
            return estRXPower
        else:
            estRXPower += (estRXPower + C.PWR_COEFF) * corrFac
        return estRXPower

    def getReceiveQuality(self):
        """
        This function calculates an estimate of the receive quality.abs

        Returns:
                The estimated receive quality for the current reception.
        """
        noiseBytes = [None] * 2
        fpAmpl2Bytes = [None] * 2
        readBytes(C.RX_FQUAL, C.STD_NOISE_SUB, noiseBytes, 2)
        readBytes(C.RX_FQUAL, C.FP_AMPL2_SUB, fpAmpl2Bytes, 2)
        noise = float(noiseBytes[0] | noiseBytes[1] << 8)
        f2 = float(fpAmpl2Bytes[0] | fpAmpl2Bytes[1] << 8)
        return f2 / noise

    def getReceiveTimestamp(self):
        """
        This function reads the receive timestamp from the register and returns it.

        Returns:
                The timestamp value of the startReceive reception.
        """
        rxTimeBytes = [0] * 5
        readBytes(C.RX_TIME, C.RX_STAMP_SUB, rxTimeBytes, 5)
        timestamp = 0
        for i in range(0, 5):
            timestamp |= rxTimeBytes[i] << (i * 8)
        timestamp = int(round(correctTimestamp(timestamp)))
        
        return timestamp

    def correctTimestamp(self, timestamp):
        """
        This function corrects the timestamp read from the RX buffer.

        Args: 
                timestamp : the timestamp you want to correct
        
        Returns: 
                The corrected timestamp.
        """
        rxPowerBase = -(getReceivePower() + 61.0) * 0.5
        rxPowerBaseLow = int(math.floor(rxPowerBase))
        rxPowerBaseHigh = rxPowerBaseLow + 1

        if rxPowerBaseLow < 0:
            rxPowerBaseLow = 0
            rxPowerBaseHigh = 0
        elif rxPowerBaseHigh > 17:
            rxPowerBaseLow = 17
            rxPowerBaseHigh = 17

        if _operationMode[C.CHANNEL_BIT] == C.CHANNEL_4 or _operationMode[C.CHANNEL_BIT] == C.CHANNEL_7:
            if _operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_16MHZ:
                if rxPowerBaseHigh < C.BIAS_900_16_ZERO:
                    rangeBiasHigh = - C.BIAS_900_16[rxPowerBaseHigh]
                else:
                    rangeBiasHigh = C.BIAS_900_16[rxPowerBaseHigh]
                rangeBiasHigh <<= 1
                if rxPowerBaseLow < C.BIAS_900_16_ZERO:
                    rangeBiasLow = -C.BIAS_900_16[rxPowerBaseLow]
                else:
                    rangeBiasLow = C.BIAS_900_16[rxPowerBaseLow]
                rangeBiasLow <<= 1
            elif _operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_64MHZ:
                if rxPowerBaseHigh < C.BIAS_900_64_ZERO:
                    rangeBiasHigh = - C.BIAS_900_64[rxPowerBaseHigh]
                else:
                    rangeBiasHigh = C.BIAS_900_64[rxPowerBaseHigh]
                rangeBiasHigh <<= 1
                if rxPowerBaseLow < C.BIAS_900_64_ZERO:
                    rangeBiasLow = -C.BIAS_900_64[rxPowerBaseLow]
                else:
                    rangeBiasLow = C.BIAS_900_64[rxPowerBaseLow]
                rangeBiasLow <<= 1
        else:
            if _operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_16MHZ:
                if rxPowerBaseHigh < C.BIAS_500_16_ZERO:
                    rangeBiasHigh = - C.BIAS_500_16[rxPowerBaseHigh]
                else:
                    rangeBiasHigh = C.BIAS_500_16[rxPowerBaseHigh]
                rangeBiasHigh <<= 1
                if rxPowerBaseLow < C.BIAS_500_16_ZERO:
                    rangeBiasLow = -C.BIAS_500_16[rxPowerBaseLow]
                else:
                    rangeBiasLow = C.BIAS_500_16[rxPowerBaseLow]
                rangeBiasLow <<= 1
            elif _operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_64MHZ:
                if rxPowerBaseHigh < C.BIAS_500_64_ZERO:
                    rangeBiasHigh = - C.BIAS_500_64[rxPowerBaseHigh]
                else:
                    rangeBiasHigh = C.BIAS_500_64[rxPowerBaseHigh]
                rangeBiasHigh <<= 1
                if rxPowerBaseLow < C.BIAS_500_64_ZERO:
                    rangeBiasLow = -C.BIAS_500_64[rxPowerBaseLow]
                else:
                    rangeBiasLow = C.BIAS_500_64[rxPowerBaseLow]
                rangeBiasLow <<= 1

        rangeBias = rangeBiasLow + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow)
        adjustmentTimeTS = rangeBias * C.DISTANCE_OF_RADIO_INV * C.ADJUSTMENT_TIME_FACTOR
        timestamp += adjustmentTimeTS
        return timestamp

    """
    Message transmission functions.
    """

    def newTransmit(self):
        """
        This function prepares the chip for a new transmission. It clears the system control register and also clears the TX latched bits in the SYS_STATUS register.
        """
        self.idle()
        self.setArray(self._sysctrl, 4, 0x00)
        self.clearTransmitStatus()
        self._deviceMode = C.TX_MODE

    def startTransmit(self):
        """
        This function configures the chip to start the transmission of the message previously set in the TX register. It sets TXSTRT bit in the system control register to begin transmission.
        """
        self.writeBytes(C.TX_FCTRL, C.NO_SUB, self._txfctrl, 5)
        self.setBit(self._sysctrl, 4, C.SFCST_BIT, False)
        self.setBit(self._sysctrl, 4, C.TXSTRT_BIT, True)
        self.writeBytes(C.SYS_CTRL, C.NO_SUB, self._sysctrl, 4)
        if self._permanentReceive:
            self.setArray(self._sysctrl, 4, 0x00)
            self._deviceMode = C.RX_MODE
            self.startReceive()
        else:
            self._deviceMode = C.IDLE_MODE

    def clearTransmitStatus(self):
        """
        This function clears the event status register at the bits related to the transmission of a message.
        """
        self.setBit(self._sysstatus, 5, C.TXFRB_BIT, True)
        self.setBit(self._sysstatus, 5, C.TXPRS_BIT, True)
        self.setBit(self._sysstatus, 5, C.TXPHS_BIT, True)
        self.setBit(self._sysstatus, 5, C.TXFRS_BIT, True)

        self.writeBytes(C.SYS_STATUS, C.NO_SUB, self._sysstatus, 5)


    def setDelay(self, delay, unit):
        """
        This function configures the chip to activate a delay between transmissions or receptions.

        Args:
                delay: The delay between each transmission/reception
                unit : The unit you want to put the delay in. Microseconds is the base unit.

        Returns:
                The timestamp's value with the added delay and antennaDelay.
        """
        if _deviceMode == C.TX_MODE:
            self.setBit(_sysctrl, 4, C.TXDLYS_BIT, True)
        elif self._deviceMode == C.RX_MODE:
            self.setBit(_sysctrl, 4, C.RXDLYE_BIT, True)

        delayBytes = [None] * 5
        sysTimeBytes = [None] * 5
        self.readBytes(C.SYS_TIME, C.NO_SUB, sysTimeBytes, 5)
        futureTimeTS = 0
        for i in range(0, 5):
            futureTimeTS |= sysTimeBytes[i] << (i * 8)
        futureTimeTS += (int)(delay * unit * C.TIME_RES_INV)

        self.setArray(delayBytes, 5, 0x00)
        for i in range(0, 5):
            delayBytes[i] = (int)(futureTimeTS >> (i * 8)) & C.MASK_LS_BYTE

        delayBytes[0] = 0
        delayBytes[1] &= C.SET_DELAY_MASK
        self.writeBytes(C.DX_TIME, C.NO_SUB, delayBytes, 5)

        futureTimeTS = 0
        for i in range(0, 5):
            futureTimeTS |= delayBytes[i] << (i * 8)

        futureTimeTS += C.ANTENNA_DELAY
        return futureTimeTS


    def clearAllStatus(self):
        """
        This function clears all the status register by writing a 1 to every bits in it. 
        """
        self.setArray(self._sysstatus, 5, 0xFF)
        self.writeBytes(C.SYS_STATUS, C.NO_SUB, self._sysstatus, 5)


    def getTransmitTimestamp(self):
        """
        This function reads the transmit timestamp from the register and returns it.

        Returns:
                The timestamp value of the last transmission.
        """
        txTimeBytes = [0] * 5
        readBytes(C.TX_TIME, C.TX_STAMP_SUB, txTimeBytes, 5)
        timeStamp = 0
        for i in range(0, 5):
            timeStamp |= int(txTimeBytes[i] << (i * 8))
        return timeStamp

    def setTimeStamp(self, data, timeStamp, index):
        """
        This function sets the specified timestamp into the data that will be sent.
        Args:
                data: The data where you will store the timestamp
                timeStamp = The timestamp's value
                index = The bit from where you will put the timestamp's value
        Returns:
                The data with the timestamp added to it
        """
        for i in range(0, 5):
            data[i+index] = int((timeStamp >> (i * 8)) & C.MASK_LS_BYTE)


    def getTimeStamp(self, data, index):
        """
        This function gets the timestamp's value written inside the specified data and returns it.
        Args:
                data : the data where you want to extract the timestamp from
                index : the index you want to start reading the data from
        Returns:
                The timestamp's value read from the given data.
        """
        timestamp = 0
        for i in range(0, 5):
            timestamp |= data[i+index] << (i*8)
        return timestamp


    def wrapTimestamp(self, timestamp):
        """
        This function converts the negative values of the timestamp due to the overflow into a correct one.
        Args :
                timestamp : the timestamp's value you want to correct.
        
        Returns:
                The corrected timestamp's value.
        """
        if timestamp < 0:
            timestamp += C.TIME_OVERFLOW
        return timestamp

            
    """
    Data functions
    """


    def getDataStr(self):
        """
        This function reads the RX buffer register to process the received message. First, it reads the length of the message in the RX frame info register, then
        read the RX buffer register and converts the ASCII values into strings.

        Returns:
                The data in the RX buffer as a string.
        """
        len = 0
        if _deviceMode == C.RX_MODE:
            rxFrameInfo = [0] * 4
            readBytes(C.RX_FINFO, C.NO_SUB, rxFrameInfo, 4)
            len = (((rxFrameInfo[1] << 8) | rxFrameInfo[0]) & C.GET_DATA_MASK)
            len = len - 2

        dataBytes = [""] * len
        readBytes(C.RX_BUFFER, C.NO_SUB, dataBytes, len)
        data = "".join(chr(i) for i in dataBytes)
        return data


    def getData(self, datalength):
        """
        This function reads a number of bytes in the RX buffer register, stores it into an array and return it.

        Args:
                datalength = The number of bytes you want to read from the rx buffer.

        Returns:
                The data read in the RX buffer as an array byte.
        """
        data = [0] * datalength
        time.sleep(0.000005)
        readBytes(C.RX_BUFFER, C.NO_SUB, data, datalength)
        return data


    def setDataStr(self, data):
        """
        This function converts the specified string into an array of bytes and calls the setData function. 

        Args:
                data: The string message the transmitter will send.
        """
        dataLength = len(data) + 1
        testData = [0] * dataLength
        for i in range(0, len(data)):
            testData[i] = ord(data[i])
        setData(testData, dataLength)


    def setData(self, data, dataLength):
        """
        This function writes the byte array into the TX buffer register to store the message which will be sent.

        Args:
                data: the byte array which contains the data to be written in the register
                dataLength: The size of the data which will be sent.
        """
        self.writeBytes(C.TX_BUFFER, C.NO_SUB, data, dataLength)
        dataLength += 2  # _frameCheck true, two bytes CRC
        self._txfctrl[0] = (dataLength & C.MASK_LS_BYTE)
        self._txfctrl[1] &= C.SET_DATA_MASK1
        self._txfctrl[1] |= ((dataLength >> 8) & C.SET_DATA_MASK2)


    def getDeviceModeInfo(self):
        """
        This function prints the various device mode operating informations such as datarate, pulse frequency, the channel used, etc
        """
        if self._operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_16MHZ:
            prf = C.PFREQ_16
        elif self._operationMode[C.PULSE_FREQUENCY_BIT] == C.TX_PULSE_FREQ_64MHZ:
            prf = C.PFREQ_64
        else:
            prf = C.PFREQ_0

        if self._operationMode[C.PREAMBLE_LENGTH_BIT] == C.TX_PREAMBLE_LEN_64:
            plen = C.PLEN_64
        elif self._operationMode[C.PREAMBLE_LENGTH_BIT] == C.TX_PREAMBLE_LEN_2048:
            plen = C.PLEN_2048
        else:
            plen = C.PLEN_0

        if self._operationMode[C.DATA_RATE_BIT] == C.TRX_RATE_110KBPS:
            dr = C.DATA_RATE_110
        else:
            dr = C.DATA_RATE_0

        ch = self._operationMode[C.CHANNEL_BIT]
        pcode = self._operationMode[C.PREAMBLE_CODE_BIT]
        print("Device mode: Data rate %d kb/s, PRF : %d MHz, Preamble: %d symbols (code %d), Channel : %d" %
            (dr, prf, plen, pcode, ch))


    """
    Helper functions
    """


    def readBytes(self, cmd, offset, data, n):
        """
        This function read n bytes from the given registers with an offset and stores them in the array given as a parameter.

        Args:
                cmd: The address of the register you want to read.
                offset: The offset for the register.
                data: The array where you want the bytes the be stored.
                n: The number of bytes you want to read from the register.
        """
        header = [None] * 3
        headerLen = 1

        if offset == C.NO_SUB:
            header[0] = C.READ | cmd
        else:
            header[0] = C.READ_SUB | cmd
            if offset < 128:
                header[1] = offset
                headerLen = headerLen + 1
            else:
                header[1] = (C.RW_SUB_EXT | offset) & C.MASK_LS_BYTE
                header[2] = offset >> 7
                headerLen = headerLen + 2

        GPIO.output(self._chipSelect, GPIO.LOW)
        for i in range(0, headerLen):
            self.spi.xfer([int(header[i])])

        for i in range(0, n):
            data[i] = self.spi.xfer([C.JUNK])[0]

        GPIO.output(self._chipSelect, GPIO.HIGH)


    def writeBytes(self, cmd, offset, data, dataSize):
        """
        This function writes n bytes from the specified array to the register given as a parameter and with an offset value.

        Args:
                cmd: The address of the register you want to write into.
                offset: The offset for the register.
                data: The array containing the data you want written.
                dataSize: The number of bytes you want to write into the register.
        """
        header = [None] * 3
        headerLen = 1

        if offset == C.NO_SUB:
            header[0] = C.WRITE | cmd
        else:
            header[0] = C.WRITE_SUB | cmd
            if offset < 128:
                header[1] = offset
                headerLen = headerLen + 1
            else:
                header[1] = (C.RW_SUB_EXT | offset) & C.MASK_LS_BYTE
                header[2] = offset >> 7
                headerLen = headerLen + 2

        GPIO.output(self._chipSelect, GPIO.LOW)
        for i in range(0, headerLen):
            self.spi.xfer([int(header[i])])

        for i in range(0, dataSize):
            if (data[i] != None):
                self.spi.xfer([int(data[i])])

        GPIO.output(self._chipSelect, GPIO.HIGH)


    def setBit(self, data, n, bit, val):
        """
        This function sets the value of a bit in an array of bytes.

        Args:
                data: The array you want to set the bit into.
                n: The number of bytes in the array
                bit: The position of the bit to be set.
                val: The boolean value to be set to the given position

        Returns:
                The modified array with the bit set according to the specified value.
        """
        idx = bit / 8
        if idx >= n:
            return
        shift = bit % 8
        if val:
            mask = 1 << shift
            data[idx] = data[idx] | mask
        else:
            mask = ~(1 << shift)
            data[idx] = data[idx] & mask
        return data


    def getBit(self, data, n, pos):
        """
        This function gets the value of a bit in an array of bytes.

        Args:
                data: The array you want to get the bit from.
                n: The number of bytes in the array.
                pos: The position of the bit to get.

        Returns:
                The bit in the array according to the position.
        """
        idx = pos / 8
        if idx >= n:
            return
        shift = pos % 8
        data[idx] = (data[idx] >> shift) & 0x01
        return data[idx]


    def setArray(self, data, size, value):
        """
        This function sets all the value in an array to the given value.

        Args:
                data: The array to be set.
                size: The size of the array.
                value: The value to be set in the array.
        """
        for i in range(0, size):
            data[i] = value


    def writeValueToBytes(self, data, val, n):
        """
        This function writes the value specified and convert it into bytes to write in the array

        Args:
                data: The array you want to write the value into.
                val: The value you want to write in the array.
                n: The size of the array. 

        Return:
                The modified array of bytes.
        """
        for i in range(0, n):
            data[i] = int(((val >> (i * 8)) & C.MASK_LS_BYTE))
        return data


    def readBytesOTP(self, address, data):
        """
        This function reads a value from the OTP memory following 6.3.3 table 13 of the user manual 

        Args:
                address: The address to read in the OTP register.
                data: The data that will store the value read. 
        """
        addressBytes = [None] * 2
        addressBytes[0] = address & C.MASK_LS_BYTE
        addressBytes[1] = (address >> 8) & C.MASK_LS_BYTE
        self.writeBytes(C.OTP_IF, C.OTP_ADDR_SUB, addressBytes, 2)
        self.writeBytes(C.OTP_IF, C.OTP_CTRL_SUB, [C.OTP_STEP2], 1)
        self.writeBytes(C.OTP_IF, C.OTP_CTRL_SUB, [C.OTP_STEP3], 1)
        self.readBytes(C.OTP_IF, C.OTP_RDAT_SUB, data, 4)
        self.writeBytes(C.OTP_IF, C.OTP_CTRL_SUB, [C.OTP_STEP5], 1)
        return data


    def convertStringToByte(self, string):
        """
        This function converts the string address used for the EUI into a byte array.

        Args:   
                string : the string you want to convert into an array of bytes

        Returns:
                The array of bytes

        """
        data = [0] * 8
        for i in range(0, 8):
            data[i] = (int(string[i * 3], 16) << 4) + int(string[i * 3 + 1], 16)
        return data


    def close(self):
        """
        This function closes the SPI connection and clean up the GPIOs.
        """
        self.spi.close()
        GPIO.cleanup()
        print("\n Close SPI")
