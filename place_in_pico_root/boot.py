# Sane Boot Defaults that won't stop the CoCo CPU

import machine

Led = machine.Pin("LED", machine.Pin.OUT)
Direction = machine.Pin(26, machine.Pin.OUT)
GHalt = machine.Pin(27, machine.Pin.OUT)
GSlenb = machine.Pin(28, machine.Pin.OUT)

Led.value(1) # LED ON
Direction.value(1) # RECEIVE FROM CoCo Data Bus; don't drive data onto the bus
GHalt.value(0) # Don't stop CPU execution by asserting HALT
GSlenb.value(0) # Don't override system RAM by asserting SLENB