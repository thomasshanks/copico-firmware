# FOR Aardvark/V1
#
# Writing 0x01 to the Control Port at $FF7A (re)starts the PIO program that
# sends bytes back to the CoCo in response Data Port reads. Each read from
# the Data Port at $FF7B will get the next letter in letter in `TEXT_TO_SEND`
# until the `TEXT_TO_SEND` string is exhausted. The PIO program will then
# no longer respond to Data Port reads until the next write of 0x01 to the
# Control Port.
#
# Copyright (c) 2024 Thomas Shanks (GitHub: thomasshanks),
#                    Phoenix B (GitHub: phoenixvox),
#                    Henry Strickland (GitHub: strickyak)
#
# Released under the standard MIT License, which is included in the LICENSE
# file in the repository.
#

import time

import machine
import micropython
import rp2
import _thread

# allow interrupts to throw errors
micropython.alloc_emergency_exception_buf(100)

OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH

NETIO_PIO = 0
CONTROL_WRITE_SM = 0
DATA_READ_SM = 1

FIRST_DATA_PIN = 0
DATA_BUS_WIDTH = 8

AardLED = machine.Pin("LED", machine.Pin.OUT)

AardWriteC = machine.Pin(10, machine.Pin.IN) # 0 = Write Control Port
AardWriteD = machine.Pin(11, machine.Pin.IN) # 0 = Write Data Port
AardReadD = machine.Pin(12, machine.Pin.IN)  # 0 = Read Data Port

AardHalt = machine.Pin(13, machine.Pin.OUT)  # 1 = Halt CoCo
AardSlenb = machine.Pin(14, machine.Pin.OUT) # 1 = Assert SLENB to CoCo
AardDir = machine.Pin(15, machine.Pin.OUT)   # 1 = IN to PicoW from CoCo
AardTrigger = machine.Pin(16, machine.Pin.OUT) # Debug output for triggering oscilloscope capture

FIRST_SIDESET_PIN = AardDir

AardLED.value(0)   # LED off
AardHalt.value(0)  # 0 = Don't HALT; run CoCo
AardSlenb.value(0) # 0 = Don't assert SLENB; CoCo device select works normally
AardDir.value(1)   # 1 = IN to PicoW from CoCo

Led = machine.Pin("LED", machine.Pin.OUT, value=0)

@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,   # Shift bits into the most significant end of the shift register
    autopush=True,                    # Automatically push the next word from input shift reg into FIFO
    push_thresh=8,                    # Push to the FIFO when 8 bits of the ISR have been produced
)
def on_control_write():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    AARD_WRITE_C = 10  # GPIO strobe for Write Control (active high)
    SM_IRQ = 4         # for State Machine communictaion

    wrap_target()                                                                                               # type: ignore

    wait(1, gpio, AARD_WRITE_C) # Wait until AARD_WRITE_C is not being asserted (to prevent false positives)    # type: ignore
    wait(0, gpio, AARD_WRITE_C) # Wait until AARD_WRITE_C is being asserted                                     # type: ignore

    in_(pins, 8) # Send contents of data bus to input FIFO so they can be printed on the PicoW console          # type: ignore

    irq(0) # Tell the CPU that we have received a Control Write strobe                                          # type: ignore

    #irq(clear, SM_IRQ) # Tell the on_data_read state machine that it can start responding to Data Read strobes  # type: ignore

    wrap()                                                                                                      # type: ignore

@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),    # Data bus pins will default to being active-high inputs
    #autopull=True,                    # Automatically pull the next word from FIFO into the output shift reg
    pull_thresh=24,                   # Pull from FIFO when 24 bits of the OSR have been consumed
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # Take bits from the least significant end of the shift register
    sideset_init=(OUT_HIGH, OUT_LOW), # 15:dir (1 = IN to PicoW from CoCo); 16:trigger (1 = trigger oscilloscope capture)
)
def on_data_read():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    AARD_READ_D = 12  # GPIO strobe for Read Data (active high)
    SM_IRQ = 4        # IRQ to wait on before responding to reads on the Data Port

    wrap_target()                                                                                               # type: ignore

    # Wait for a signal from the control port indicating that we can start responding to Data Read strobes
    #irq(block, SM_IRQ)                                                                                          # type: ignore

    pull(ifempty) # Get the next word from the FIFO and put it into the output shift register                   # type: ignore

    wait(1, gpio, AARD_READ_D) # Wait until AARD_READ_D is not being asserted (to prevent false positives)      # type: ignore
    wait(0, gpio, AARD_READ_D) # Wait until AARD_READ_D is being asserted                                       # type: ignore

    out(pindirs, 8) # Set the data bus pins to be outputs on the PicoW side                                     # type: ignore

    out(pins, 8).side(0b10) # trigger=1; direction=0 (OUT to CoCo)                                              # type: ignore
    wait(1, gpio, AARD_READ_D) # Wait until AARD_READ_D is no longer asserted                                   # type: ignore
    nop().side(0b01) # trigger=0; direction=1 (IN from CoCo)                                                    # type: ignore

    out(pindirs, 8) # Set the data bus pins to be inputs on the PicoW side                                      # type: ignore

    wrap()                                                                                                      # type: ignore

# Remove all existing PIO programs from PIO 0
rp2.PIO(NETIO_PIO).remove_program()

sm_control_write = rp2.StateMachine(
    (NETIO_PIO << 3) + CONTROL_WRITE_SM, # which state machine in pio0
    on_control_write,
    freq=125_000_000,
    in_base=FIRST_DATA_PIN,
)
sm_control_write.active(True)

sm_data_read = rp2.StateMachine(
    (NETIO_PIO << 3) + DATA_READ_SM, # which state machine in pio0
    on_data_read,
    freq=125_000_000,
    out_base=FIRST_DATA_PIN,
    sideset_base=FIRST_SIDESET_PIN,
)
sm_data_read.active(True)

# This is a global variable that will be used to control when the
# `sendbytes_task` enqueues bytes to send to the CoCo
sendbytes_should_stop = True

def handler(sm):
    global sendbytes_should_stop

    control_byte = sm_control_write.get()
    print(f'Control Byte Received: {control_byte} ({chr(control_byte)})')
    if control_byte == 0x00:
        sendbytes_should_stop = True
        if sm_data_read.active():
            #sm_data_read.active(False)

            # Empty the TX FIFO by throwing away the contents of the OSR five times
            for _ in range(5):
                sm_data_read.restart()
    elif control_byte == 0x01:
        sendbytes_should_stop = False
        #sm_data_read.active(True)

sm_control_write.irq(handler=handler) # TODO: What are trigger=0|1 and hard=True|False for?

BYTES_TO_SEND = b"Hello world from PIO! We all live in a yellow submarine!!!\r" 

def sendbytes_task(sm_data_read, bytes_to_send):
    """ Enqueue bytes into TX FIFO of Data Read PIO State Machine so that it can send them to the CoCo."""
    global sendbytes_should_stop

    while True:
        for byte in bytes_to_send:
            # Keep trying to send the next `char` in `text` until we have
            # successfully sent it (or we are asked to stop sending)
            while not sendbytes_should_stop:
                # Don't try to send the next byte unless the TX FIFO has
                # room for it
                if sm_data_read.tx_fifo() < 4:
                    # Wrap the byte to send with the pindirs values to
                    # configure before (0xFF) and after (0x00)
                    word_to_send = 0x0000FF | (byte << 8)
                    print(f'Queuing {hex(word_to_send)} ("{chr(byte)}" surrounded by the pindirs)')
                    sm_data_read.put(word_to_send)

                    # Go on to the next `char` in the `text` string
                    break
            else: # if sendbytes_should_stop:
                # TODO: Wait on a _thread.lock() that indicates it is time
                # to start sending text again instead of spinning in the
                # `while True:` loop

                # Exit the `for char in text:` loop
                break
        else: # if we've reached the end of the text
            # then stop sending, but don't stop the state machine
            sendbytes_should_stop = True

_thread.start_new_thread(sendbytes_task, (sm_data_read, BYTES_TO_SEND))

## Blink LED at 1 Hz
AardLED.value(1)
try:
    while True:
        AardLED.value(1)
        time.sleep(0.2)
        AardLED.value(0)
        time.sleep(0.8)
        if sm_data_read.tx_fifo():
            print(f'TX FIFO: {sm_data_read.tx_fifo()}')
finally:
    del sm_data_read
    del sm_control_write
    print('PIO and DMA freed')
    import gc; gc.collect()
    print('gc complete')
    AardLED.value(0)
