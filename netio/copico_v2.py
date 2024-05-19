
import machine

LED_PIN = "LED"

#GSCS_N_PIN = 16 # Disabled due to position of jumper (GA8 vs GSCS_N)
#Q_CLOCK_PIN = 17 # Disabled due to position of jumper (GA9 vs GQ)
#HALT_PIN = 18 # Disabled due to position of jumper (A10/NETIO_SEL1_N vs GHALT)
#SLENB_PIN = 19 # Disabled due to position of jumper (A11/GETIO_SEL2_N vs GSLENB)
#RESETN_PIN = 20  # Disabled due to position of jumper (A12/NETIO_SEL3_N vs GRESETN)
#NETIO_SEL_N_PIN = 21 # Disabled due to position of jumper (A13/NETIO_SEL4_N vs GNETIO_SEL_N)

NETIO_STROBE1_N_PIN = 18
CONTROL_STROBE_N_PIN = NETIO_STROBE1_N_PIN
#WRITE_CONTROL_STROBE_PIN = # Read/write signal is now connected to RP2040 directly
NETIO_STROBE2_N_PIN = 19
DATA_STROBE_N_PIN = NETIO_STROBE2_N_PIN
#WRITE_DATA_STROBE_PIN = # Read/write signal is now connected to RP2040 directly
#READ_DATA_STROBE_PIN = # Read/write signal is now connected to RP2040 directly
NETIO_STROBE3_N_PIN = 20
NETIO_STROBE4_N_PIN = 21

READ_NOT_WRITE_PIN = 22
CTSN_PIN = 26
E_CLOCK_PIN = 27
DRIVE_DATA_BUS_PIN = 28
#READ_NOT_WRITE_PIN = 28  # Disabled due to position of jumper (DRIVE_DATA_BUS vs GREAD_NOT_WRITE)

#TRIGGER_PIN =

FIRST_DATA_PIN = 0
FIRST_ADDRESS_PIN = 8
NUM_ADDRESS_PINS = 10

FIRST_SIDESET_PIN = DRIVE_DATA_BUS_PIN

class BoardPins:
    def __init__(self):
        self.led = machine.Pin("LED", machine.Pin.OUT)

        #self.scsn = machine.Pin(GSCS_N_PIN, machine.Pin.IN) # Disabled due to position of jumper (GA8 vs GSCS_N)
        #self.q_clock = machine.Pin(Q_CLOCK_PIN, machine.Pin.IN) # Disabled due to position of jumper (GA9 vs GQ)
        #self.halt = machine.Pin(HALT_PIN, machine.Pin.OUT)  # 1 = Halt CoCo # Disabled due to position of jumper (A10/NETIO_SEL1_N vs GHALT)
        #self.slenb = machine.Pin(SLENB_PIN, machine.Pin.OUT) # 1 = Assert SLENB to CoCo # Disabled due to position of jumper (A11/GETIO_SEL2_N vs GSLENB)
        #self.reset_n = machine.Pin(RESETN_PIN, machine.Pin.IN) # Disabled due to position of jumper (A12/NETIO_SEL3_N vs GRESETN)
        #self.netio_sel_n = machine.Pin(NETIO_SEL_N_PIN, machine.Pin.IN) # Disabled due to position of jumper (A13/NETIO_SEL4_N vs GNETIO_SEL_N)


        self.netio_strobe1_n = machine.Pin(NETIO_STROBE1_N_PIN, machine.Pin.IN)
        self.control_strobe_n = self.netio_strobe1_n
        #self.write_control_strobe = machine.Pin(WRITE_CONTROL_STROBE_PIN, machine.Pin.IN) # 0 = Write Control Port
        self.netio_strobe2_n = machine.Pin(NETIO_STROBE2_N_PIN, machine.Pin.IN)
        self.data_strobe_n = self.netio_strobe2_n
        #self.write_data_strobe = machine.Pin(WRITE_DATA_STROBE_PIN, machine.Pin.IN) # 0 = Write Data Port
        #self.read_data_strobe = machine.Pin(READ_DATA_STROBE_PIN, machine.Pin.IN)  # 0 = Read Data Port
        self.netio_strobe3_n = machine.Pin(NETIO_STROBE3_N_PIN, machine.Pin.IN)
        self.netio_strobe4_n = machine.Pin(NETIO_STROBE4_N_PIN, machine.Pin.IN)

        self.read_not_write = machine.Pin(READ_NOT_WRITE_PIN, machine.Pin.IN)
        self.ctsn = machine.Pin(CTSN_PIN, machine.Pin.IN)
        self.e_clock = machine.Pin(E_CLOCK_PIN, machine.Pin.IN)
        self.drive_data_bus = machine.Pin(DRIVE_DATA_BUS_PIN, machine.Pin.OUT)   # 1 = IN to PicoW from CoCo

        #self.trigger = machine.Pin(TRIGGER_PIN, machine.Pin.OUT) # Debug output for triggering oscilloscope capture
