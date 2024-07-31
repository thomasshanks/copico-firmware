#!/usr/bin/env python3
#
# Copyright (c) 2024 Thomas Shanks (GitHub: thomasshanks),
#                    Phoenix B (GitHub: phoenixvox),
#                    Henry Strickland (GitHub: strickyak)
#
# Released under the standard MIT License, which is included in the LICENSE
# file in the repository.
#
# SPDX-License-Identifier: MIT
#
# --------------------- CoPiCo Proof of Concept Firmware --------------------
#
# net4 is a lowlevel test of a lowlevel protocol
# synchronizing one tx stream and one rx stream
# of bytes between Coco bus and Pi Pico, using
# all four netio ports and all four state machines
# in PIO instance 0.
#
# it is slow because this is Python, and a Python interrupt
# handler gets invoked for every data byte.
#
# ----------------------------------------------------------------------------
#
# This program is a proof of concept for the CoPiCo project.
#
# There are four netio ports (R/W from Coco point of view):
#   net1 = ControlPort (write only)
#   net2 = StatusPort (read only)
#   net3 = TxPort (read only)
#   net4 = RxPort (write only)
#
# At the high level, the Coco can write one data byte (the CoPiCo will Rx)
# or read one data byte (the Pico will Tx).  
# In this file, we will use the names Rx and Tx from the point
# of view of the Pi Pico, which is the opposite of the Coco's point of view.
# That is because the Pi Pico documentation uses the Pico point of view.
# Read and Write are terms from the Coco's CPU's point of view.

# For the Coco to write one data byte for the Pico to Receive,
# the coco should
#   Write data byte to RxPort
#   Write 1 to ControlPort
#   Read from StatusPort until nonzero.

# For the Coco to read one data byte that the pico Transmits,
# the coco should
#   Write 2 to ControlPort
#   Read from StatusPort until nonzero.
#   Read data byte from TxPort

# That way, even though slow MicroPython code will get invoked to do each
# operation, the Coco stays synchronized with the Pico python program.

#from __future__ import annotations # Not supported on MicroPython

#import atexit # Not supported on MicroPython
import time

import micropython
import rp2
import _thread

import copico_v2 as board
import wifi
import uasyncio


#from abc import ABC, abstractmethod # Not supported on MicroPython

# A special constant that is assumed to be True by 3rd party static type checkers. It is False at runtime.
TYPE_CHECKING = False

if TYPE_CHECKING:
    from typing import Optional

import dma_defs
import pio_defs

OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH

NETIO_PIO = 0
CONTROL_SM = 0
STATUS_SM = 1
TX_SM = 2
RX_SM = 3

DATA_BUS_WIDTH = 8

MSG = b'SO RAISE YOUR JOYSTICKS, RAISE YOUR KEYBOARDS, LET CRTS ILLUMINATE THE WAY, WITH EVERY LINE, WITH EVERY BYTE, FOR COCO, WE PLEDGE ALLEGIANCE, THIS DAY!\n'
msg_i = 0

def NextMessageByte():
    global msg_i
    z = MSG[msg_i] & 255
    msg_i += 1
    if msg_i >= len(MSG): msg_i = 0
    return z

def WrapTxByteWithPindirs(x):
    return 0x0000_00FF | ((255&x)<<8)

@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,   # Shift bits into the most significant end of the shift register
    autopush=True,                    # Automatically push the next word from input shift reg into FIFO
    push_thresh=8,                    # Push to the FIFO when 8 bits of the ISR have been produced
)
def on_control_write():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    CTRL_STROBE_N = 18 # board.CONTROL_STROBE_N_PIN # GPIO strobe for Control Register (active high)

    wrap_target()                                                                                               # type: ignore

    wait(0, gpio, CTRL_STROBE_N) # Wait until CTRL_STROBE_N drops, mid-cycle.
    wait(1, gpio, CTRL_STROBE_N) # Wait until CTRL_STROBE_N rises, at end of cycle.

    in_(pins, 8) # Send contents of data bus to Control FIFO.
    irq(0) # Tell the CPU that we have received a Control Write strobe                                          # type: ignore

    wrap()                                                                                                      # type: ignore

@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),    # Data bus pins will default to being active-high inputs
    autopull=False,                   # Wait for explicit PULL instructions.
    pull_thresh=24,                   # Pull from FIFO when 24 bits of the OSR have been consumed
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # Take bits from the least significant end of the shift register
    sideset_init=(OUT_HIGH),          # 15:drive_data_bus (1 = IN to PicoW from CoCo)
#   sideset_init=(OUT_HIGH, OUT_LOW), # 15:drive_data_bus (1 = IN to PicoW from CoCo); 16:trigger (1 = trigger oscilloscope capture)
)
def on_status_read():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    STATUS_STROBE_N = 19

    pull(block)   # get default no-ready-yet status bytes
    out(x, 24)    # save them in X, which is used when PULL NOBLOCK fails.

    wrap_target()                                                                                               # type: ignore

    wait(0, gpio, STATUS_STROBE_N) # Wait until strobe drops mid-cycle.
    pull(noblock)  # puts X in OSR if pull fails.
    out(pindirs, 8)         # Data bus outputs.
    out(pins, 8).side(0b0)  # drive_data_bus=0 (OUT to CoCo)
    wait(1, gpio, STATUS_STROBE_N) # Wait until strobe is over, at the end of cycle.

    set(y, 10)
    label("delay")
    jmp(y_dec, "delay") [7]

    out(pindirs, 8).side(0b1)  # Revert to inputs.  Stop driving data bus.

    wrap()


@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),    # Data bus pins will default to being active-high inputs
    autopull=False,                   # Wait for explicit PULL instructions.
    pull_thresh=24,                   # Pull from FIFO when 24 bits of the OSR have been consumed
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # Take bits from the least significant end of the shift register
    sideset_init=(OUT_HIGH),          # 15:drive_data_bus (1 = IN to PicoW from CoCo)
#   sideset_init=(OUT_HIGH, OUT_LOW), # 15:drive_data_bus (1 = IN to PicoW from CoCo); 16:trigger (1 = trigger oscilloscope capture)
)
def on_tx_read():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    TX_STROBE_N = 20

    pull(block)   # get default no-ready-yet TX bytes
    out(x, 24)    # save them in X, which is used when PULL NOBLOCK fails.

    wrap_target()                                                                                               # type: ignore

    wait(0, gpio, TX_STROBE_N) # Wait until strobe drops mid-cycle.
    pull(noblock)  # puts X in OSR if pull fails.
    out(pindirs, 8)         # Data bus outputs.
    out(pins, 8).side(0b0)  # drive_data_bus=0 (OUT to CoCo)
    wait(1, gpio, TX_STROBE_N) # Wait until strobe is over, at the end of cycle.

    set(y, 10)
    label("delay")
    jmp(y_dec, "delay") [7]

    out(pindirs, 8).side(0b1)  # Revert to inputs.  Stop driving data bus.

    wrap()


@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,   # Shift bits into the most significant end of the shift register
    autopush=True,                    # Automatically push the next word from input shift reg into FIFO
    push_thresh=8,                    # Push to the FIFO when 8 bits of the ISR have been produced
)
def on_rx_write():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    RX_STROBE_N = 21

    wrap_target()                                                                                               # type: ignore

    wait(0, gpio, RX_STROBE_N) # Wait until RX_STROBE_N drops, mid-cycle.
    wait(1, gpio, RX_STROBE_N) # Wait until RX_STROBE_N rises, at end of cycle.

    in_(pins, 8) # Send contents of data bus to RX FIFO.

    wrap()                                                                                                      # type: ignore


class PIOStateMachineManager:
    def __init__(self):
        self.sm_control_write = None
        self.sm_data_read = None

    def __enter__(self):
        self.set_up_state_machines()
        return self

    def __exit__(self, *args):
        self.cleanup()

    def set_up_state_machines(self) -> None:
        # Remove all existing PIO programs from PIO 0
        rp2.PIO(NETIO_PIO).remove_program()

        # "_read" and "_write" are from the point of view of the 6809 CPU data bus.
        # "_tx_" and "_rx_" are from the point of view of the Pico.

        self.sm_control_write = rp2.StateMachine(
            (NETIO_PIO << 3) + CONTROL_SM, # which state machine in pio0
            on_control_write,
            freq=125_000_000,
            in_base=board.FIRST_DATA_PIN,
        )

        self.sm_status_read = rp2.StateMachine(
            (NETIO_PIO << 3) + STATUS_SM, # which state machine in pio0
            on_status_read,
            freq=125_000_000,
            out_base=board.FIRST_DATA_PIN,
            sideset_base=board.FIRST_SIDESET_PIN,
        )

        self.sm_tx_read = rp2.StateMachine(
            (NETIO_PIO << 3) + TX_SM, # which state machine in pio0
            on_tx_read,
            freq=125_000_000,
            out_base=board.FIRST_DATA_PIN,
            sideset_base=board.FIRST_SIDESET_PIN,
        )

        self.sm_rx_write = rp2.StateMachine(
            (NETIO_PIO << 3) + RX_SM, # which state machine in pio0
            on_rx_write,
            freq=125_000_000,
            in_base=board.FIRST_DATA_PIN,
        )

        self.sm_status_read.put(WrapTxByteWithPindirs(0))
        self.sm_tx_read.put(WrapTxByteWithPindirs(0))

        def handler(sm):
            if not self.sm_control_write:
                raise ValueError("Control Write State Machine has not been set up")

            control_byte = self.sm_control_write.get()
            print(f'Control Byte Received: {hex(control_byte)}')
            if control_byte == 1:  # Rx
                # the data byte x must already be on the rx fifo.
                fifo = self.sm_rx_write.rx_fifo()
                x = self.sm_rx_write.get()
                print("rx: %d  fifo=%d" % (x, fifo))
                self.sm_status_read.put(WrapTxByteWithPindirs(x | 128))
            elif control_byte == 2:  # Tx
                x = NextMessageByte()
                fifo = self.sm_tx_read.tx_fifo()
                print("tx: %d <%s> fifo=%d" % (x, chr(x), fifo))
                self.sm_tx_read.put(WrapTxByteWithPindirs(x))
                self.sm_status_read.put(WrapTxByteWithPindirs(15))   # Status 15
            else:
                print("Undefined control byte: %d" % control_byte)

        # TODO: What are the `trigger=0|1` and `hard=True|False` options on `.irq(..)` for?
        self.sm_control_write.irq(handler=handler)

    def start_state_machines(self):
        if not self.sm_control_write or not self.sm_status_read:
            raise ValueError("State machines have not been set up")
        if not self.sm_tx_read or not self.sm_rx_write:
            raise ValueError("State machines have not been set up")

        # Empty all FIFOs by throwing away the contents of the OSR five times
        for _ in range(5):
            self.sm_tx_read.restart()
            self.sm_rx_write.restart()
            self.sm_status_read.restart()
            self.sm_control_write.restart()

        self.sm_status_read.active(True)
        self.sm_tx_read.active(True)
        self.sm_rx_write.active(True)
        self.sm_control_write.active(True)

    def stop_state_machines(self):
        if self.sm_control_write:
            self.sm_control_write.active(False)
        if self.sm_status_read:
            self.sm_status_read.active(False)
        if self.sm_rx_write:
            self.sm_rx_write.active(False)
        if self.sm_tx_read:
            self.sm_tx_read.active(False)

    def cleanup(self):
        self.stop_state_machines()
        del self.sm_control_write
        del self.sm_status_read
        del self.sm_rx_write
        del self.sm_tx_read

        # Remove all existing PIO programs from PIO 0
        rp2.PIO(NETIO_PIO).remove_program()

        print('PIO freed')

async def blink(led):
    while True:
        led.on()
        await uasyncio.sleep_ms(1000)
        led.off()
        await uasyncio.sleep_ms(1000)

async def blinker(led):
    uasyncio.create_task(blink(led))
    r, w = uasyncio.open_connection("10.23.23.23", 2321)
    await uasyncio.sleep_ms(20_000)

if __name__ == "__main__":
    # allow interrupts to throw errors
    micropython.alloc_emergency_exception_buf(200)

    pins = board.BoardPins()

    pins.led.value(0)   # LED off
    #pins.halt.value(0)  # 0 = Don't HALT; run CoCo
    #pins.slenb.value(0) # 0 = Don't assert SLENB; CoCo device select works normally
    pins.drive_data_bus.value(1)   # 1 = IN to PicoW from CoCo

    try:
        with PIOStateMachineManager() as sm_mgr:
                sm_mgr.start_state_machines()

                uasyncio.run(blinker(pins.led))

                # Blink LED at 1 Hz
                pins.led.value(1)

                while True:
                    pins.led.value(1)
                    time.sleep(0.2)
                    pins.led.value(0)
                    time.sleep(0.8)

    finally:
        pins.led.value(0)
        del pins

        import gc; gc.collect()
        print('gc complete')
