# Copyright (c) 2024 Thomas Shanks; released under the standard MIT License
# Portions Copyright (c) 2024 Henry Strickland (GitHub: strickyak), included under the MIT License
#
# This is a proof of concept showing that the RP2040 can respond to memory-mapped I/O within the
# ranges 0xFF68-0xFF6F and 0xFF78-0xFF7F, as detected by the 74LS133-based address decoding logic.

import time

import machine
import rp2

OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH

FIRST_DATA_PIN = 0
DATA_BUS_WIDTH = 8
FIRST_ADDRESS_PIN = 8
NUM_ADDRESS_PINS = 10
READ_NOT_WRITE_PIN = 18
RESETN_PIN = 19
NETIO_SEL_N_PIN = 20
CTSN_PIN = 21
E_CLOCK_PIN = 22
DRIVE_DATA_BUS_N_PIN = 26
HALT_PIN = 27
SLENB_PIN = 28

# 0xFF=pindirs(make data bus pins outputs) 0x39=ASCII('9') 00=pindirs(back to being inputs)
HELLO_WORLD_DATA = 0x000039FF

ON_NETIO_SEL_PROG_PIO_ID = 0
ON_NETIO_SEL_PROG_SM_ID_ON_PIO = 0
SM_IRQ_FOR_NETIO_RESET = rp2.PIO.IRQ_SM0

Led = machine.Pin("LED", machine.Pin.OUT, value=0)

Data = [machine.Pin(FIRST_DATA_PIN + i, machine.Pin.IN) for i in range(DATA_BUS_WIDTH)]
Address = [machine.Pin(FIRST_ADDRESS_PIN + i, machine.Pin.IN) for i in range(NUM_ADDRESS_PINS)]

ReadNotWrite = machine.Pin(READ_NOT_WRITE_PIN, machine.Pin.IN)

# TODO: Reset the NetIO function when the CoCo is reset
ResetN = machine.Pin(RESETN_PIN, machine.Pin.IN)

# NETIO_SEL_N is the signal that tells us that the CoCo is addressing the Network I/O ranges 
# (0xFF68-0xFF6F and 0xFF78-0xFF7F), as detected by the 74LS133-based address decoding logic.
NetIOSelN = machine.Pin(NETIO_SEL_N_PIN, machine.Pin.IN)

CtsN = machine.Pin(CTSN_PIN, machine.Pin.IN)
EClock = machine.Pin(E_CLOCK_PIN, machine.Pin.IN)

# 'DriveDataBus' controls the direction of the 74LVC245 bidirectional (Low Voltage CMOS)
# level-shifting buffer between the Color Computer's Data Bus (D0-D7) and the Pico's GPIO0-GPIO7.
# A value of 1 means the RP2040 will hear the value coming IN from the CoCo's data bus, while a
# value of 0 means that we will drive the RP2040's output OUT TO the CoCo's data bus. The default
# is IN (1); this is what we want until it's time to drive our value out onto the CoCo data bus.
DriveDataBusN = machine.Pin(DRIVE_DATA_BUS_N_PIN, machine.Pin.OUT, value=1)

# We only want to use these pins for hardware debugging or for "spoonfeeding" the CoCo's 6809 CPU,
# neither of which are applicable to the NetIO functionality. We still need to set the pin
# direction correctly for these signals, though.
Halt = machine.Pin(HALT_PIN, machine.Pin.OUT, value=0)
Slenb = machine.Pin(SLENB_PIN, machine.Pin.OUT, value=0)

@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),    # 0-7: D0-D7
    sideset_init=(OUT_HIGH, ),        # DriveDataBus is active low, so we start with it high
    in_shiftdir=rp2.PIO.SHIFT_RIGHT,  # We will shift in 10 bits at a time
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # We will shift out 8 bits at a time
    autopush=True,
    push_thresh=10,                   # 10 bits of address
    autopull=True,
    pull_thresh=24,                   # 8 bits of pindirs, 8 bits of data, 8 bits of pindirs
)
def on_netio_sel_prog():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    DATA_BUS_WIDTH = 8
    NUM_ADDRESS_PINS = 10
    NETIO_SEL_N_PIN = 20
    E_CLOCK_PIN = 22

    # Wait until we see an address in the NetIO ranges (0xFF68-0xFF6F or 0xFF78-0xFF7F)
    wait(1, gpio, NETIO_SEL_N_PIN)                                                                     # type: ignore
    wait(0, gpio, NETIO_SEL_N_PIN)                                                                     # type: ignore
    
    wait(1, gpio, E_CLOCK_PIN) # wait until E is high before we start looking for the falling edge     # type: ignore

    # Latch the address on the falling edge of E clock
    ##in_(pins, NUM_ADDRESS_PINS)                                                                        # type: ignore

    # TODO: Extract A4 from the ISR and use it to determine if the CoCo is accessing whichever of
    # the two Network I/O ranges (0xFF68-0xFF6F or 0xFF78-0xFF7F) we are configured to respond to.

    # TODO: Extract A0 and A1 from the ISR and use them to determine which of the four addresses
    # in the Network I/O range we are responding to.

    # Jump to "do_netio_read" if the CoCo is reading from the Network I/O range, or continue into
    # "do_netio_write" if CoCo is writing to the Network I/O range
    jmp(pin, "do_netio_read")   # The JMP pin is READ_NOT_WRITE_PIN; 0 means write, 1 means read       # type: ignore

    # BEGIN do_netio_write
    label("do_netio_write")                                                                            # type: ignore
    irq(0)                      # Send interrupt to tell CPU to (re)start the TCP stream               # type: ignore
    # TODO: Wait for the TCP stream to be (re)started before continuing
    jmp("end_of_netio")                                                                                # type: ignore
    # END do_netio_write

    # BEGIN do_netio_read
    label("do_netio_read")                                                                             # type: ignore

    # TODO: Don't block here if data is not available; do something sane instead                       # type: ignore
    out(pindirs, DATA_BUS_WIDTH)  .side(0)  # DRIVE_DATA_BUS_N=0=OUT                                   # type: ignore
    out(pins, DATA_BUS_WIDTH)                                                                          # type: ignore
    wait(1, gpio, E_CLOCK_PIN)              # wait until E hi                                          # type: ignore
    wait(0, gpio, E_CLOCK_PIN)              # wait until E lo                                          # type: ignore
    out(pindirs, DATA_BUS_WIDTH)  .side(1)  # DRIVE_DATA_BUS_N=1=IN                                    # type: ignore

    label("end_of_netio")                                                                              # type: ignore
    # END do_netio_read

sm = rp2.StateMachine(
    ON_NETIO_SEL_PROG_SM_ID_ON_PIO, # which state machine in pio0
    on_netio_sel_prog,
    freq=125_000_000,
    in_base=Address[0],
    out_base=Data[0],
    jmp_pin=ReadNotWrite,
    sideset_base=DriveDataBusN,
)

# TODO: Consider using a DMA-based NetIO buffer
def netio_write_irq_handler(pio):
    print('Handling NetIO Write')

    if sm.tx_fifo() == 0:
        print('Putting "9" on the output FIFO')
        sm.put(HELLO_WORLD_DATA) # Put '9' character, along with pindirs, on the output FIFO

    print('Done handling NetIO Write')

# Set up the handler for the NetIO Write IRQ
rp2.PIO(ON_NETIO_SEL_PROG_PIO_ID).irq(
    netio_write_irq_handler,
    trigger=SM_IRQ_FOR_NETIO_RESET,
    ##hard=True # TODO: What does "hard=True" do?
)

sm.put(HELLO_WORLD_DATA) # Put '9' character, along with pindirs, on the output FIFO
Led.value(1)
sm.active(True)

try:
    while True:
        pass
except KeyboardInterrupt:
    print('Deactivating PIO 0 SM 1')
    sm.active(False)
    Halt.value(0)
    Slenb.value(0)
    DriveDataBusN.value(1)
    Led.value(0)
    print("DONE")
