# FOR Aardvark/V1

# Any write to the Control Port at $FF7A allows output.
# If output is allowed, reads from the Data Port at $FF7B will get 26, then 25, then 24... to 0.
# When it reaches 0, output is no longer allowed, until the next write to Control Port.

# Copyright (c) 2024 Thomas Shanks; released under the standard MIT License
# Portions Copyright (c) 2024 Henry Strickland (GitHub: strickyak), included under the MIT License

import time

import machine
import rp2

from machine import Pin


AardLED = Pin("LED", Pin.OUT)

AardHalt = Pin(13, Pin.OUT)
AardSlenb = Pin(14, Pin.OUT)
AardDir = Pin(15, Pin.OUT)

AardWriteC = Pin(10, Pin.IN)
AardWriteD = Pin(11, Pin.IN)
AardReadD = Pin(12, Pin.IN)

AardLED.value(0)
AardHalt.value(0)
AardSlenb.value(0)
AardDir.value(1)  # 1 = IN to PicoW

OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH

FIRST_DATA_PIN = 0
DATA_BUS_WIDTH = 8
FIRST_SIDESET_PIN = 13 # AardHalt

Led = machine.Pin("LED", machine.Pin.OUT, value=0)

@rp2.asm_pio(
)
def on_control_write():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    AARD_WRITE_C = 10  # GPIO strobe for Write Control (active high)
    SM_IRQ = 4         # for State Machine communictaion

    wrap_target()
    wait(1, gpio, AARD_WRITE_C)
    irq(clear, SM_IRQ)
    wait(0, gpio, AARD_WRITE_C)
    wrap()

@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),    # 0-7: D0-D7
    sideset_init=(OUT_LOW, OUT_LOW, OUT_HIGH, OUT_LOW),  # 13:Halt 14:Slenb 15:dir 16:trigger
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # We will shift out 8 bits at a time
)
def on_data_read():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    AARD_READ_D = 12  # GPIO strobe for Read Data (active high)
    SM_IRQ = 4         # for State Machine communictaion

    # Wait for a singal from the control port.
    wrap_target()
    irq(block, SM_IRQ)
    set(y, 26)      # Letter 'Z' in VDG Text Mode

    # Wait for Data Port read.
    label("wait_data_port_read")
    wait(1, gpio, AARD_READ_D)                                                                     # type: ignore

    set(x, 0)
    set(x, ~x) # , op=invert)      # set x := 255 (meaning OUTPUT to data bus)
    mov(pindirs, x)   .side(0b1000) # trigger=1 direction=0=OUT Slenb=no Halt=no
    mov(pins, y)
    
    wait(0, gpio, AARD_READ_D) # wait until strobe falls  # type: ignore
    set(x, 0)
    mov(pindirs, x)   .side(0b0100) # trigger=0 direction=1=IN Slenb=no Halt=no

    # After y reaches 0 ('@' on VDG Text Screen), don't respond to Data Read strobes.
    jmp(y_dec, "wait_data_port_read")
    # until we get the signal on the Control Port.
    wrap()

pio0 = rp2.PIO(0)

pio0.add_program(on_control_write)
sm_control_write = pio0.state_machine(
    0, # which state machine in pio0
    on_control_write,
    freq=125_000_000,
)
sm_control_write.active(True)

pio0.add_program(on_data_read)
sm_data_read = pio0.state_machine(
    1, # which state machine in pio0
    on_data_read,
    freq=125_000_000,
    out_base=FIRST_DATA_PIN,
    sideset_base=FIRST_SIDESET_PIN,
)
#sm_data_read.active(True)

# Blink LED at 1 Hz
while True:
	AardLED.value(1)
	time.sleep(0.5)
	AardLED.value(0)
	time.sleep(0.5)
