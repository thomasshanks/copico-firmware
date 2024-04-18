# FOR Aardvark/V1

# Any write to the Control Port at $FF7A allows output.
# If output is allowed, reads from the Data Port at $FF7B will get 26, then 25, then 24... to 0.
# When it reaches 0, output is no longer allowed, until the next write to Control Port.

# Copyright (c) 2024 Phoenix Brown under MIT License
# Copyright (c) 2024 Henry Strickland (GitHub: strickyak) under the MIT License
# Copyright (c) 2024 Thomas Shanks; released under the standard MIT License

import time

import machine
import rp2
import _thread

OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH

AardLED = machine.Pin("LED", machine.Pin.OUT)

AardWriteC = machine.Pin(10, machine.Pin.IN) # 0 = Write Control Port
AardWriteD = machine.Pin(11, machine.Pin.IN) # 0 = Write Data Port
AardReadD = machine.Pin(12, machine.Pin.IN)  # 0 = Read Data Port

AardHalt = machine.Pin(13, machine.Pin.OUT)  # 1 = Halt CoCo
AardSlenb = machine.Pin(14, machine.Pin.OUT) # 1 = Assert SLENB to CoCo
AardDir = machine.Pin(15, machine.Pin.OUT)   # 1 = IN to PicoW from CoCo
AardTrigger = machine.Pin(16, machine.Pin.OUT) # Debug output for triggering oscilloscope capture

AardLED.value(0)   # LED off
AardHalt.value(0)  # 0 = Don't HALT; run CoCo
AardSlenb.value(0) # 0 = Don't assert SLENB; CoCo device select works normally
AardDir.value(1)   # 1 = IN to PicoW from CoCo

FIRST_DATA_PIN = 0
DATA_BUS_WIDTH = 8
FIRST_SIDESET_PIN = AardDir

Led = machine.Pin("LED", machine.Pin.OUT, value=0)

@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT, # Shift bits into the most significant end of the shift register
)
def on_control_write():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    AARD_WRITE_C = 10  # GPIO strobe for Write Control (active high)
    SM_IRQ = 4         # for State Machine communictaion

    wrap_target()

    wait(1, gpio, AARD_WRITE_C) # Wait until AARD_WRITE_C is not being asserted (to prevent false positives)
    wait(0, gpio, AARD_WRITE_C) # Wait until AARD_WRITE_C is being asserted

    irq(clear, SM_IRQ) # Tell the on_data_read state machine that it can start responding to Data Read strobes

    wrap()

@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),    # Data bus pins will default to being active-high inputs
    autopull=True,                    # Automatically pull the next word from FIFO into the output shift reg
    pull_thresh=24,                   # Pull from FIFO when 24 bits of the OSR have been consumed
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # Take bits from the least significant end of the shift register
    sideset_init=(OUT_HIGH, OUT_LOW), # 15:dir (1 = IN to PicoW from CoCo); 16:trigger (1 = trigger oscilloscope capture)
    autopush=True,                    # Automatically push the next word from input shift reg into FIFO
)
def on_data_read():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    AARD_READ_D = 12  # GPIO strobe for Read Data (active high)
    SM_IRQ = 4        # IRQ to wait on before responding to reads on the Data Port

    wrap_target()
    
    irq(block, SM_IRQ) # Wait for a signal from the control port indicating that we can start responding to Data Read strobes

    set(y, 26) # Respond to this many Data Read strobes before stopping until the next Control Write

    # Loop that responds to Data Port reads
    label("wait_data_port_read")
    
    wait(1, gpio, AARD_READ_D) # Wait until AARD_READ_D is not being asserted (to prevent false positives)
    wait(0, gpio, AARD_READ_D) # Wait until AARD_READ_D is being asserted

    #mov(osr, invert(null))
    out(pindirs, 8) # Set the data bus pins to be outputs on the PicoW side

    out(pins, 8).side(0b10) # trigger=1; direction=0 (OUT to CoCo)
    #in_(pins, 8) # Send the current value of y to the input FIFO so that we can print what we sent to the PicoW console
    wait(1, gpio, AARD_READ_D) # Wait until AARD_READ_D is no longer asserted
    nop().side(0b01) # trigger=0; direction=1 (IN from CoCo)
    
    #mov(osr, null)
    out(pindirs, 8) # Set the data bus pins to be inputs on the PicoW side
    
    irq(0) # Tell the CPU that we have finished responding to a Data Read strobe
    
    # After y reaches 0, we will stop responding to Data Read strobes until the next Control Port Write
    jmp(y_dec, "wait_data_port_read")

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
sm_data_read.active(True)

def handler(pio):
    print(' handler called ')

pio0.irq(handler)

# Send a string to the CoCo
def sendchar_task(sm_data_read, text):
    for char in text:
        word_to_send = 0x0000FF + ((ord(char) & 0xFF) << 8)
        print(f'Queuing {hex(word_to_send)} ("{char}" surrounded by the pindirs)')
        sm_data_read.put(word_to_send)

text = "Hello world from PIO!"
_thread.start_new_thread(sendchar_task, (sm_data_read, text))

# Blink LED at 1 Hz
while True:
    AardLED.value(1)
    time.sleep(0.2)
    #read_byte = sm_data_read.get()
    #print(f'{read_byte} ({chr(read_byte)})')
    AardLED.value(0)
    time.sleep(0.8)
