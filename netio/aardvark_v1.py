import machine

LED_PIN = "LED"

WRITE_CONTROL_STROBE_PIN = 10
WRITE_DATA_STROBE_PIN = 11
READ_DATA_STROBE_PIN = 12

HALT_PIN = 13
SLENB_PIN = 14
DRIVE_DATA_BUS_PIN = 15

#TRIGGER_PIN = 16

FIRST_DATA_PIN = 0

FIRST_SIDESET_PIN = DRIVE_DATA_BUS_PIN

class BoardPins:
    def __init__(self):
        self.led = machine.Pin("LED", machine.Pin.OUT)

        self.write_control_strobe = machine.Pin(WRITE_CONTROL_STROBE_PIN, machine.Pin.IN) # 0 = Write Control Port
        self.write_data_strobe = machine.Pin(WRITE_DATA_STROBE_PIN, machine.Pin.IN) # 0 = Write Data Port
        self.read_data_strobe = machine.Pin(READ_DATA_STROBE_PIN, machine.Pin.IN)  # 0 = Read Data Port

        self.halt = machine.Pin(HALT_PIN, machine.Pin.OUT)  # 1 = Halt CoCo
        self.slenb = machine.Pin(SLENB_PIN, machine.Pin.OUT) # 1 = Assert SLENB to CoCo
        self.drive_data_bus = machine.Pin(DRIVE_DATA_BUS_PIN, machine.Pin.OUT)   # 1 = IN to PicoW from CoCo

        #self.trigger = machine.Pin(TRIGGER_PIN, machine.Pin.OUT) # Debug output for triggering oscilloscope capture
