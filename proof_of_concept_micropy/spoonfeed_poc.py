# This is strickyak's spoonfeed demo modified for the CoPiCo

import time

import machine, rp2

FIRST_DATA_PIN = 0
NUM_DATA_PINS = 8
FIRST_ADDRESS_PIN = 8
NUM_ADDRESS_PINS = 10 # Actually 9 since we're reallocating GPIO 17 to triggering the oscilloscope
READ_NOT_WRITE_PIN = 18
RESETN_PIN = 19
NETIO_SEL_N_PIN = 20
CTSN_PIN = 21
E_CLOCK_PIN = 22
DRIVE_DATA_BUS_PIN = 26
HALT_PIN = 27
SLENB_PIN = 28

Led = machine.Pin("LED", machine.Pin.OUT)
Halt = machine.Pin(HALT_PIN, machine.Pin.OUT)
Slenb = machine.Pin(SLENB_PIN, machine.Pin.OUT)
Dir = machine.Pin(DRIVE_DATA_BUS_PIN, machine.Pin.OUT)
Trigger = machine.Pin(17, machine.Pin.OUT) # Make sure to remove JP2 and connect oscilloscope to center machine.Pin
ResetN = machine.Pin(RESETN_PIN, machine.Pin.IN)
EClock = machine.Pin(E_CLOCK_PIN, machine.Pin.IN)

Led.value(1)
Halt.value(0)
Slenb.value(0)
Dir.value(1)
Trigger.value(0)

OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH
@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),   # 0-7: D0-D7
    sideset_init=(OUT_HIGH, OUT_LOW, OUT_HIGH, OUT_LOW),  # 13:Halt 14:Slenb 15:dir 16:trigger
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # 8 bits at a time
    autopull=True,
    pull_thresh=32,
    )
def onreset_prog():
    E_CLOCK_PIN = 22 # Constants used in PIO ASM must be defined inside the program, not global
    wait(0, gpio, E_CLOCK_PIN) # synchronize on a full pulse of E.
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo

    # Unhalt at the beginning of a Cycle, so we can count cycles.
    # "direction" is the direction of the bidirectional (low voltage cmos) buffer
    # between the Coco Data Bus (D0-D7) and the Pico's GPIO0-GPIO7.
    # Out means out to the coco.  In means in from the Coco.
    # The default is IN, unless we really mean to be writing to the Coco.
    set(x, 3)                   .side(0b100) # 3 means loop 4x # direction=1=IN Slenb=no Halt=no # Unhalts the M6809
    
    # Count four dead cycles (includes the one in which we unhalted).
    label("four_times")
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    jmp(x_dec, "four_times")
    
    # Output the HIGH then the LOW byte of the reset vector we want.
    out(pindirs, 8)             .side(0b010) # direction=0=OUT Slenb=yes Halt=no
    out(pins, 8)                   # output HIGH byte of Reset Vector (8 bits from OSR)
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    out(pins, 8)                   # output LOW byte of Reset Vector (8 bits from OSR)
    wait(1, gpio, E_CLOCK_PIN)  .side(0b011) # wait until E hi, Halt=yes (to halt before first instruction)
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    out(pindirs, 8)             .side(0b101) # direction=1=IN, Slenb=no, Halt=yes

    # Get stuck here until the main routine re-inits this state machine.
    label("loop_forever")
    jmp("loop_forever")
    
@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),   # 0-7: D0-D7
    sideset_init=(OUT_HIGH, OUT_LOW, OUT_HIGH, OUT_LOW),  # 13:Halt 14:Slenb 15:dir 16:trigger
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # 8 bits at a time
    autopull=True,
    pull_thresh=32,
    )
def ldd_immediate_std_extended_prog():
    """This PIO program is designed to unhalt, to output 6 bytes of opcodes
       in the first six cycles, then to allow 3 more cycles to pass,
       and then halt again.  That matches a sequence like this:
         LDD #$4845  (3 instruction bytes; 3 cycles)
	     STD $0400   (3 instruction bytes; 6 cycles)
    """
    E_CLOCK_PIN = 22 # Constants used in PIO ASM must be defined inside the program, not global
    wait(0, gpio, E_CLOCK_PIN) # synchronize on a full pulse of E.
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo

    # Three dead cycles.  We release halt during these.
    set(x, 2)              # Loop three times, so count down with X=2.
    label("three_times")
    wait(1, gpio, E_CLOCK_PIN)  .side(0b0100) # wait until E hi # trigger=0 direction=1=IN Slenb=no Halt=no # Unhalts the M6809
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    jmp(x_dec, "three_times")

    set(x, 5)         # Loop six times, so count down with X=5.
    out(pindirs, 8)             .side(0b0010) # trigger=0 direction=0=OUT Slenb=yes Halt=no
    label("six_times")
    out(pins, 8)                   # output HIGH byte of Reset Vector (8 bits from OSR)
    wait(1, gpio, E_CLOCK_PIN)  .side(0b0010) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN)  .side(0b1010) # wait until E lo
    jmp(x_dec, "six_times")

    # Three more cycles will execute, but we don't have to wait for them.
    # The CPU will halt after those cycles, at the end of the instruction.
    out(pindirs, 8)   .side(0b0101) # trigger=0 direction=1=IN Slenb=no Halt=yes

    # Get stuck here until the main routine re-inits this state machine.
    label("loop_forever")
    jmp("loop_forever")

print("Step2: waiting for RESET.  ")
while ResetN.value()==1: pass  # wait for drop
print("got RESET.  ")
Led.value(0)
Halt.value(1)                  # halt while resetting
time.sleep(0.1)                     # debounce
print("debounced.  ")
while ResetN.value()==0: pass  # wait for ResetN to release
print("RESET gone.  ")
time.sleep(0.5)                     # debounce and wait to sync on Halt
print("SLEPT half a second.  ")

pio0 = rp2.PIO(0)
pio0.add_program(onreset_prog)

sm1 = pio0.state_machine(
    1,  # which state machine in pio0
    onreset_prog,
    freq=125_000_000,
    sideset_base=Halt,
    out_base=0,
)
# FF=outputs A027=reset_vector 00=inputs
sm1.put(0x0027a0ff)

Led.value(1)
sm1.active(True)
print("Activated onreset prog.  Deactivating.\n")
sm1.active(False)
pio0.remove_program(onreset_prog)
pio0.add_program(ldd_immediate_std_extended_prog)

MSG = 'SO RAISE YOUR JOYSTICKS, RAISE YOUR KEYBOARDS, LET CRTS ILLUMINATE THE WAY, WITH EVERY LINE, WITH EVERY BYTE, FOR COCO, WE PLEDGE ALLEGIANCE, THIS DAY!'
MSG = MSG+' ' if 1&len(MSG) else MSG
PairsOfWords = []
LDD_IMMEDIATE, STD_EXTENDED = 0xCC, 0xFD

for i in range(len(MSG)//2):
    b1, b2 = 63&ord(MSG[i+i]), 63&ord(MSG[i+i+1])
    w1 = (b2<<24) | (b1<<16) | (LDD_IMMEDIATE<<8) | 0xFF
    w2 = ((i+i)<<16) | STD_EXTENDED
    PairsOfWords.append((w1, w2, b1, b2))


for (w1, w2, b1, b2) in PairsOfWords:
  sm2 = pio0.state_machine(2) # which state machine in pio
  sm2.init(ldd_immediate_std_extended_prog,
    freq=125_000_000,
    sideset_base=Halt,
    out_base=0,
  )
  # sm2.put(0x5958CCff)  # ff=outputs CC=LDD_immediate $58='X' $59='Y'
  # sm2.put(0x000000FD)  # FD=STD_extended 00=inputs
  sm2.put(w1)  # ff=outputs CC=LDD_immediate $58='X' $59='Y'
  sm2.put(w2)  # FD=STD_extended 00=inputs
  sm2.active(True)
  print("(%x, %x, %x, %x) Activated sm2. Deactivating." % (w1, w2, b1, b2))
  sm2.active(False)


pio0.remove_program(ldd_immediate_std_extended_prog)
print("DONE")
while True:
    pass
# ldd_immediate_std_extended_prog
