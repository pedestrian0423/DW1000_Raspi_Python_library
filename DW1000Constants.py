"""
This python module is used to store every constants relevant for the DW1000 module and scripts.
"""

###########################################################################################
#### From DW1000Constants.h
###########################################################################################

# Time stamp byte length
LEN_STAMP = 5

# DW1000 TX/RX Modes
IDLE_MODE = 0x00
RX_MODE = 0x01
TX_MODE = 0x02

# Used for SPI ready w/o actual writes
JUNK = 0x00

# No sub-address for register write
NO_SUB = 0xFF

# Device id register
DEV_ID = 0x00
LEN_DEV_ID = 4

# Extended unique identifier register
EUI = 0x01
LEN_EUI = 8

# PAN identifier, short address register
PANADR = 0x03
LEN_PANADR = 4

# Device configuration register
SYS_CFG = 0x04
LEN_SYS_CFG = 4
    # --
FFEN_BIT = 0
FFBC_BIT = 1
FFAB_BIT = 2
FFAD_BIT = 3
FFAA_BIT = 4
FFAM_BIT = 5
FFAR_BIT = 6
    # --
DIS_STXP_BIT = 18
PHR_MODE_SUB = 16
LEN_PHR_MODE_SUB = 2
    # -- System configuration register bits, see 7.2.6 of User Manual.
DIS_DRXB_BIT = 12
HIRQ_POL_BIT = 9
RXM110K_BIT = 22
RXAUTR_BIT = 29


# Device control register
SYS_CTRL = 0x0D
LEN_SYS_CTRL = 4
    # --
WAIT4RESP_BIT = 7
    # -- System control register bits, see 7.2.15 of User Manual
SFCST_BIT = 0
TXSTRT_BIT = 1
TXDLYS_BIT = 2
TRXOFF_BIT = 6
RXENAB_BIT = 8
RXDLYE_BIT = 9

# System event status register
SYS_STATUS = 0x0F
LEN_SYS_STATUS = 5
CPLOCK_BIT = 1
AAT_BIT = 3
    # -- System event status register bits, see 7.2.17 of User Manual
TXFRB_BIT = 4
TXPRS_BIT = 5
TXPHS_BIT = 6
TXFRS_BIT = 7
    # -- System event status register bits, see 7.2.17 of User Manual
LDEDONE_BIT = 10
RXPHE_BIT = 12
RXDFR_BIT = 13
RXFCG_BIT = 14
RXFCE_BIT = 15
RXRFSL_BIT = 16
RXRFTO_BIT = 17
RXPTO_BIT = 21
RXSFDTO_BIT = 26
LDEERR_BIT = 18
    # --
RFPLL_LL_BIT = 24
CLKPLL_LL_BIT = 25

# system event mask register
# NOTE: uses the bit definitions of SYS_STATUS (below 32)
SYS_MASK = 0x0E
LEN_SYS_MASK = 4

# System time counter
SYS_TIME = 0x06
LEN_SYS_TIME = LEN_STAMP

# RX timestamp register
RX_TIME = 0x15
LEN_RX_TIME = 14
    # -- RX_TIME subregisters
RX_STAMP_SUB = 0x00
FP_AMPL1_SUB = 0x07
    # --
LEN_RX_STAMP = LEN_STAMP
LEN_FP_AMPL1 = 2

# RX frame quality
RX_FQUAL = 0x12
LEN_RX_FQUAL = 8
    # -- RX_FQUAL subregisters
STD_NOISE_SUB = 0x00
FP_AMPL2_SUB = 0x02
FP_AMPL3_SUB = 0x04
#PP_AMPL3_SUB = 0x04
CIR_PWR_SUB = 0x06
    # --
LEN_STD_NOISE = 2
LEN_FP_AMPL2 = 2
LEN_FP_AMPL3 = 2
LEN_CIR_PWR = 2

# TX timestamp register
TX_TIME = 0x17
LEN_TX_TIME = 10
    # -- TX_TIME subregisters
TX_STAMP_SUB = 0
    # --
LEN_TX_STAMP = LEN_STAMP

# Timing register (for delayed RX/TX)
DX_TIME = 0x0A
LEN_DX_TIME = LEN_STAMP

# Transmit data buffer
TX_BUFFER = 0x09
LEN_TX_BUFFER = 1024
LEN_UWB_FRAMES = 127
LEN_EXT_UWB_FRAMES = 1023

# RX frame info
RX_FINFO = 0x10
LEN_RX_FINFO = 4

# Receive data buffer
RX_BUFFER = 0x11
LEN_RX_BUFFER = 1024

# Transmit control
TX_FCTRL = 0x08
LEN_TX_FCTRL = 5

# Channel control
CHAN_CTRL = 0x1F
LEN_CHAN_CTRL = 4
    # -- Channel control register bits, see 7.2.32 of user manual
DWSFD_BIT = 17
TNSSFD_BIT = 20
RNSSFD_BIT = 21

# user-defined SFD
USR_SFD = 0x21
LEN_USR_SFD = 41
SFD_LENGTH_SUB = 0x00
LEN_SFD_LENGTH = 1

# OTP control (for LDE micro code loading only)
OTP_IF = 0x2D
    # -- OTP_IF subregisters
OTP_ADDR_SUB = 0x04
OTP_CTRL_SUB = 0x06
OTP_RDAT_SUB = 0x0A
    # --
LEN_OTP_ADDR = 2
LEN_OTP_CTRL = 2
LEN_OTP_RDAT = 4

# AGC_TUNE1/2 (for re-tuning only)
AGC_TUNE = 0x23
    # -- AGC_CTRL subregisters
AGC_TUNE1_SUB = 0x04
AGC_TUNE2_SUB = 0x0C
AGC_TUNE3_SUB = 0x12
    # --
LEN_AGC_TUNE1 = 2
LEN_AGC_TUNE2 = 4
LEN_AGC_TUNE3 = 2

# DRX_TUNE2 (for re-tuning only)
DRX_TUNE = 0x27
    # -- DRX_CONF subregisters
DRX_TUNE0b_SUB = 0x02
DRX_TUNE1a_SUB = 0x04
DRX_TUNE1b_SUB = 0x06
DRX_TUNE2_SUB = 0x08
DRX_TUNE4H_SUB = 0x26
    # --
LEN_DRX_TUNE0b = 2
LEN_DRX_TUNE1a = 2
LEN_DRX_TUNE1b = 2
LEN_DRX_TUNE2 = 4
LEN_DRX_TUNE4H = 2

# LDE_CFG1 (for re-tuning only)
LDE_IF = 0x2E
    # -- LDE_IF subregisters
LDE_CFG1_SUB = 0x0806
LDE_RXANTD_SUB = 0x1804
LDE_CFG2_SUB = 0x1806
LDE_REPC_SUB = 0x2804
    # --
LEN_LDE_CFG1 = 1
LEN_LDE_RXANTD = 2
LEN_LDE_CFG2 = 2
LEN_LDE_REPC = 2

# TX_POWER (for re-tuning only)
TX_POWER = 0x1E
LEN_TX_POWER = 4

# RF_CONF (for re-tuning only)
RF_CONF = 0x28
    # -- RF_CONF subregisters
RF_RXCTRLH_SUB = 0x08
RF_TXCTRL_SUB = 0x0C
    # --
LEN_RF_RXCTRLH = 1
LEN_RF_TXCTRL = 4

# TX_CAL (for re-tuning only)
TX_CAL = 0x2A
    # -- TX_CAL subregisters
TC_PGDELAY_SUB = 0x0B
    # --
LEN_TC_PGDELAY = 1
TC_SARC = 0x00
TC_SARL = 0x03

# FS_CTRL (for re-tuning only)
FS_CTRL = 0x2B
    # -- FS_CTRL subregisters
FS_PLLCFG_SUB = 0x07
FS_PLLTUNE_SUB = 0x0B
FS_XTALT_SUB = 0x0E
    # --
LEN_FS_PLLCFG = 4
LEN_FS_PLLTUNE = 1
LEN_FS_X_TALT = 1

# AON (#### what is this??)
AON = 0x2C
AON_WCFG_SUB = 0x00
LEN_AON_WCFG = 2
ONW_LDC_BIT = 6
ONW_LDD0_BIT = 12
AON_CTRL_SUB = 0x02
LEN_AON_CTRL = 1
RESTORE_BIT = 0
SAVE_BIT = 1
UPL_CFG_BIT = 2

AON_CFG0_SUB = 0x06
LEN_AON_CFG0 = 4
SLEEP_EN_BIT = 0
WAKE_PIN_BIT = 1
WAKE_SPI_BIT = 2
WAKE_CNT_BIT = 3

# PMSC
PMSC = 0x36
    # -- Registers offset
PMSC_CTRL0_SUB = 0x00
    # --
PMSC_CTRL1_SUB = 0x04
PMSC_LEDC_SUB = 0x28
LEN_PMSC_CTRL0 = 4
LEN_PMSC_CTRL1 = 4
LEN_PMSC_LEDC = 4
GPDCE_BIT = 18
KHZCLKEN_BIT = 23
BLNKEN = 8

ATXSLP_BIT = 11
ARXSLP_BIT = 12

# TX_ANTD Antenna delays
TX_ANTD = 0x18
LEN_TX_ANTD = 2

# GPIO
GPIO_CTRL = 0x26
GPIO_MODE_SUB = 0x00
LEN_GPIO_MODE = 4

MSGP0 = 6
MSGP1 = 8
MSGP2 = 10
MSGP3= 12
MSGP4 = 14
MSGP5 = 16
MSGP6 = 18
MSGP7 = 20
MSGP8 = 22

GPIO_MODE = 0
LED_MODE = 1

###########################################################################################
#### FROM DW1000.h
###########################################################################################

# transmission/reception bit rate
TRX_RATE_110KBPS = 0x00
TRX_RATE_850KBPS = 0x01
TRX_RATE_6800KBPS = 0x02

# transmission pulse frequency
# 0x00 is 4MHz, but receiver in DW1000 does not support it (!??)
TX_PULSE_FREQ_16MHZ = 0x01
TX_PULSE_FREQ_64MHZ = 0x02

# preamble length (PE + TXPSR bits)
TX_PREAMBLE_LEN_64 = 0x01
TX_PREAMBLE_LEN_128 = 0x05
TX_PREAMBLE_LEN_256 = 0x09
TX_PREAMBLE_LEN_512 = 0x0D
TX_PREAMBLE_LEN_1024 = 0x02
TX_PREAMBLE_LEN_1536 = 0x06
TX_PREAMBLE_LEN_2048 = 0x0A
TX_PREAMBLE_LEN_4096 = 0x03

# PAC size
PAC_SIZE_8 = 8
PAC_SIZE_16 = 16
PAC_SIZE_32 = 32
PAC_SIZE_64 = 64

# Channel of operation
CHANNEL_1 = 1
CHANNEL_2 = 2
CHANNEL_3 = 3
CHANNEL_4 = 4
CHANNEL_5 = 5
CHANNEL_7 = 7

# Preamble codes
PREAMBLE_CODE_16MHZ_1 = 1
PREAMBLE_CODE_16MHZ_2 = 2
PREAMBLE_CODE_16MHZ_3 = 3
PREAMBLE_CODE_16MHZ_4 = 4
PREAMBLE_CODE_16MHZ_5 = 5
PREAMBLE_CODE_16MHZ_6 = 6
PREAMBLE_CODE_16MHZ_7 = 7
PREAMBLE_CODE_16MHZ_8 = 8
PREAMBLE_CODE_64MHZ_9 = 9
PREAMBLE_CODE_64MHZ_10 = 10
PREAMBLE_CODE_64MHZ_11 = 11
PREAMBLE_CODE_64MHZ_12 = 12
PREAMBLE_CODE_64MHZ_17 = 17
PREAMBLE_CODE_64MHZ_18 = 18
PREAMBLE_CODE_64MHZ_19 = 19
PREAMBLE_CODE_64MHZ_20 = 20

# frame length settings
FRAME_LENGTH_NORMAL = 0x00
FRAME_LENGTH_EXTENDED = 0x03

# pre-defined modes of operation (3 bytes for data rate, pulse frequency and preamble length)
    # -- DW1000 Modes of operation
    # -- ONLY MODE_LONGDATA_RANGE_LOWPOWER and MODE_LONGDATA_RANGE_ACCURACY is used in original python library.
MODE_LONGDATA_RANGE_LOWPOWER = [TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048]
MODE_SHORTDATA_FAST_LOWPOWER = [TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128]
MODE_LONGDATA_FAST_LOWPOWER = [TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024]
MODE_SHORTDATA_FAST_ACCURACY = [TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128]
MODE_LONGDATA_FAST_ACCURACY = [TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024]
MODE_LONGDATA_RANGE_ACCURACY = [TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048]

# Register is 6 bit, 7 = write, 6 = sub-addressing, 5-0 = register value
# Total header with sub-addressing can be 15-bit.
    # -- SPI operations
WRITE = 0x80
WRITE_SUB = 0xC0
READ = 0x00
READ_SUB = 0x40
RW_SUB_EXT = 0x80

# clocks available
    # -- Clocks
AUTO_CLOCK = 0x00
XTI_CLOCK = 0x01
PLL_CLOCK = 0x02

# range bias tables (500/900 MHz band, 16/64 MHz PRF), -61 to -95 dBm.
    # -- correct time stamp
BIAS_500_16_ZERO = 10
BIAS_500_64_ZERO = 8
BIAS_900_16_ZERO = 7
BIAS_900_64_ZERO = 7

# range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
BIAS_500_16 = [198, 187, 179, 163, 143, 127, 109, 84, 59, 31, 0, 36, 65, 84, 97, 106, 110, 112]
BIAS_500_64 = [110, 105, 100, 93, 82, 69, 51, 27, 0, 21, 35, 42, 49, 62, 71, 76, 81, 86]
BIAS_900_16 = [137, 122, 105, 88, 69, 47, 25, 0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197]
BIAS_900_64 = [147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142]


###########################################################################################
#### From DW1000Time.h
###########################################################################################

# Time resolution in micro-seconds of time based registers/values.
# Each bit in a timestam counts for a period of approx. 15.65 ps
    # -- time resolution in micro seconds of time based registers
TIME_RES = 0.000015650040064103
TIME_RES_INV = 63897.6

# Speed of radio waves [m/s] * timestamp resolution [~15.65 ps] of DW1000
DISTANCE_OF_RADIO = 0.0046917639786159
DISTANCE_OF_RADIO_INV = 213.139451293

# Timestamp byte length - 40 bit -> 5 byte
LENGTH_TIMESTAMP = 5

# Timer/counter overflow (40 bits) -> 4 overflow approx. every 17.2 seconds
TIME_OVERFLOW = 0x10000000000
TIME_MAX = 0xffffffffff

# Time factors (relative to [us]) for setting delayed transceive
# TODO use non float (?)
SECONDS = 1000000
MILLISECONDS = 1000
MICROSECONDS = 1
NANOSECONDS = 0.001

###########################################################################################
#### From DW1000Mac.h
###########################################################################################

FC_1_BLINK = 0xC5
FC_1 = 0x41
FC_2 = 0x8C
FC_2_SHORT = 0x88

PAN_ID_1 = 0xCA
PAN_ID_2 = 0xDE

# Frame length
LONG_MAC_LEN = 15
SHORT_MAC_LEN = 9

###########################################################################################
#### From DW1000Device.h
###########################################################################################

INACTIVITY_TIME = 1000

###########################################################################################
#### From DW1000Ranging.h
###########################################################################################

# Messages used in the ranging protocol
    # -- Message types
POLL = 0
POLL_ACK = 1
RANGE = 2
RANGE_REPORT = 3
RANGE_FAILED = 255
BLINK = 4
RANGING_INIT = 5

LEN_DATA = 90

# Default Pin for module:
DEFAULT_RST_PIN = 9
DEFAULT_SPI_SS_PIN = 10

# Default value
# in ms
DEFAULT_RESET_PERIOD = 200
# in us
DEFAULT_REPLY_DELAY_TIME = 7000

# Default timer delay
DEFAULT_TIMER_DELAY = 80

###########################################################################################
#### NOT SORTED
###########################################################################################

# Delay
INIT_DELAY = 0.000005
PMSC_CONFIG_DELAY = 0.00015

# frame length settings
FRAME_LENGTH_NORMAL = 0x00
FRAME_LENGTH_EXTENDED = 0x03

# Transmission/reception bit rate
DATA_RATE_0 = 0
DATA_RATE_110 = 110

# Transmission pulse frequency
PFREQ_0 = 0
PFREQ_16 = 16
PFREQ_64 = 64

# Preamble length
PLEN_0 = 0
PLEN_64 = 64
PLEN_2048 = 2048

# Registers ID
AGC_CTRL = 0x23
DRX_CONF = 0x27
LDE_CTRL = 0x2E

# RX_FQUAL subregisters
OTP_XTAL_ADDRESS = 0x01E

# System event mask register bits, see 7.2.16 of User Manual
MAAT_BIT = 3
MTXFRS_BIT = 7
MLDEDONE_BIT = 10
MRXPHE_BIT = 12
MRXDFR_BIT = 13
MRXFCG_BIT = 14
MRXFCE_BIT = 15
MRXRFSL_BIT = 16
MLDEERR_BIT = 18

# Enable clock masks
ENABLE_CLOCK_MASK1 = 0xFE
ENABLE_CLOCK_MASK2 = 0xFC

# PMSC operation data bytes
# SOFTRESET operations, see SOFTRESET section in 7.2.50.1 of the user manual
SOFT_RESET_SYSCLKS = 0x01
SOFT_RESET_CLEAR = 0x00
SOFT_RESET_SET = 0xF0
# Register access operations to load LDE microcode, see table 4 in 2.5.5.10 of the user manual
LDE_L1STEP1 = 0x01
LDE_L1STEP2 = 0x03
LDE_L2STEP1 = 0x00
LDE_L2STEP2 = 0x80
LDE_L3STEP1 = 0x00
LDE_L3STEP2 = 0x02

# Read OTP memory operation data bytes
OTP_STEP2 = 0x03
OTP_STEP3 = 0x01
OTP_STEP5 = 0x00

# EnableMode masks
ENABLE_MODE_MASK1 = 0x83
ENABLE_MODE_MASK2 = 0xFC
ENABLE_MODE_MASK3 = 0xF3
ENABLE_MODE_MASK4 = 0xC3
# sfdLength
SFD_LENGTH_850KBPS = 0x10
SFD_LENGTH_6800KBPS = 0x08
SFD_LENGTH_OTHER = 0x40
# preacode masks
PREACODE_MASK1 = 0x1F
PREACODE_MASK2 = 0x3F
PREACODE_MASK3 = 0x07

# Set delay masks
SET_DELAY_MASK = 0xFE

# get data masks
GET_DATA_MASK = 0x03FF

# set data masks
SET_DATA_MASK1 = 0xE0
SET_DATA_MASK2 = 0x03

# Tune operations
# AGC_TUNE1, see 7.2.36.3 of the user manual
AGC_TUNE1_DEFAULT_OP = 0x889B
AGC_TUNE1_16MHZ_OP = 0x8870
# AGC_TUNE2, see 7.2.63.5 of the user manual
AGC_TUNE2_OP = 0x2502A907
# AGC_TUNE3, see 7.2.36.7 of the user manual
AGC_TUNE3_OP = 0x0035
# DRX_TUNE0b, see 7.2.40.2 of the user manual
DRX_TUNE0b_110KBPS_NOSTD_OP = 0x0016
DRX_TUNE0b_850KBPS_NOSTD_OP = 0x0006
DRX_TUNE0b_6800KBPS_STD_OP = 0x0001
# DRX_TUNE1a, see 7.2.40.3 of the user manual
DRX_TUNE1a_16MHZ_OP = 0x0087
DRX_TUNE1a_64MHZ_OP = 0x008D
# DRX_TUNE1b, see 7.2.40.4 of the user manual
DRX_TUNE1b_M1024 = 0x0064
DRX_TUNE1b_L1024 = 0x0020
DRX_TUNE1b_64 = 0x0010
# DRX_TUNE2, see 7.2.40.5 of the user manual
DRX_TUNE2_8_16MHZ = 0x311A002D
DRX_TUNE2_8_64MHZ = 0x313B006B
DRX_TUNE2_16_16MHZ = 0x331A0052
DRX_TUNE2_16_64MHZ = 0x333B00BE
DRX_TUNE2_32_16MHZ = 0x351A009A
DRX_TUNE2_32_64MHZ = 0x353B015E
DRX_TUNE2_64_16MHZ = 0x371A011D
DRX_TUNE2_64_64MHZ = 0x373B0296
# LDE_CFG2, see 7.2.47.6 of the user manual
LDE_CFG2_16MHZ_OP = 0x1607
LDE_CFG2_64MHZ_OP = 0x0607
# DRX_TUNE4H, see 7.2.40.10 of the user manual
DRX_TUNE4H_64 = 0x0010
DRX_TUNE4H_128 = 0x0028
# RF_RXCTRLH, see 7.2.41.3 of the user manual
RF_RXCTRLH_1235 = 0xD8
RF_RXCTRLH_147 = 0xBC
# RF_TXCTRL, see 7.2.41.4 of the user manual
RF_TXCTRL_1 = 0x00005C40
RF_TXCTRL_2 = 0x00045CA0
RF_TXCTRL_3 = 0x00086CC0
RF_TXCTRL_4 = 0x00045C80
RF_TXCTRL_5 = 0x001E3FE0
RF_TXCTRL_7 = 0x001E7DE0
# TC_PGDELAY, see 7.2.43.4 of the user manual
TC_PGDELAY_1 = 0xC9
TC_PGDELAY_2 = 0xC2
TC_PGDELAY_3 = 0xC5
TC_PGDELAY_4 = 0x95
TC_PGDELAY_5 = 0xC0
TC_PGDELAY_7 = 0x93
# FS_PLLCFG, see 7.2.44.2 of the user manual
FS_PLLCFG_1 = 0x09000407
FS_PLLCFG_24 = 0x08400508
FS_PLLCFG_3 = 0x08401009
FS_PLLCFG_57 = 0x0800041D
# FS_PLLTUNE, see 7.2.44.3 of the user manual
FS_PLLTUNE_1 = 0x1E
FS_PLLTUNE_24 = 0x26
FS_PLLTUNE_3 = 0x5E
FS_PLLTUNE_57 = 0xBE
# TX_POWER, see 7.2.31.3 table 19 of the user manual
TX_POWER_12_16MHZ = 0x75757575
TX_POWER_12_64MHZ = 0x67676767
TX_POWER_3_16MHZ = 0x6F6F6F6F
TX_POWER_3_64MHZ = 0x8B8B8B8B
TX_POWER_4_16MHZ = 0x5F5F5F5F
TX_POWER_4_64MHZ = 0x9A9A9A9A
TX_POWER_5_16MHZ = 0x48484848
TX_POWER_5_64MHZ = 0x85858585
TX_POWER_7_16MHZ = 0x92929292
TX_POWER_7_64MHZ = 0xD1D1D1D1
# LDE_CFG1, see 2.5.5.4 of the user manual
LDE_CFG1_OP = 0xD
# LDE_REPC, see 7.2.47.7 table 48 of the user manual
LDE_REPC_1AND2 = 0x5998
LDE_REPC_3AND8 = 0x51EA
LDE_REPC_4 = 0x428E
LDE_REPC_5 = 0x451E
LDE_REPC_6 = 0x2E14
LDE_REPC_7 = 0x8000
LDE_REPC_9 = 0x28F4
LDE_REPC_1017 = 0x3332
LDE_REPC_111321 = 0x3AE0
LDE_REPC_12 = 0x3D70
LDE_REPC_14161819 = 0x35C2
LDE_REPC_15 = 0x2B84
LDE_REPC_20 = 0x47AE
LDE_REPC_2224 = 0x3850
LDE_REPC_23 = 0x30A2
# Tune masks
TUNE_MASK1 = 0x1F
TUNE_MASK2 = 0x60
TUNE_OPERATION = 0x10

# signal power in the first path, see 4.7.1 of the user manual and receive signal power, see 4.7.2 of the user manual
A_16MHZ = 115.72
CORRFAC_16MHZ = 2.3334
A_64MHZ = 121.74
CORRFAC_64MHZ = 1.1667
TWOPOWER17 = 131072
PWR_COEFF = 88
PWR_COEFF2 = 10.0

# Time
ANTENNA_DELAY_RASPI = 16390
ANTENNA_DELAY = 16470

RESET_PERIOD = 250

# Bits/Bytes operation
MASK_LS_BYTE = 0xFF
MASK_LS_2BITS = 0x03
MASK_LS_2BYTES = 0xFFFF
MASK_NIBBLE = 0x0F

# Correct Time stamp
ADJUSTMENT_TIME_FACTOR = 0.001

# Operation mode bits
DATA_RATE_BIT = 0
PULSE_FREQUENCY_BIT = 1
PAC_SIZE_BIT = 2
PREAMBLE_LENGTH_BIT = 3
CHANNEL_BIT = 4
PREAMBLE_CODE_BIT = 5

RECEIVER = 0
TRANSMITTER = 1
