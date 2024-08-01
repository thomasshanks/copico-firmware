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
# Use the same four I/O ports and the "sock" object from wifi.
#
# There are four netio ports (R/W from Coco point of view):
#   net1 = ControlPort (write only)
#   net2 = StatusPort (read only)
#   net3 = TxPort (read only)
#   net4 = RxPort (write only)
#
# For Coco to write N bytes to the picow (and out to the TCP
# server) (where 1 <= N <= 100):
#   Poke N to ControlPort
#   Peek StatusPort until it is nonzero
#   Poke the N bytes to RxPort, but wait on StatusPort first
#
# For Coco to read N bytes from the pico (that came from the
# TCP server) (where 1 <= N <= 100):
#   Poke (100+N) to ControlPort
#   Peek StatusPort until it is nonzero
#   Peek the N bytes from TxPort, and wait on StatusPort after
#
# This could use DMA (TODO!) and could use Interrupts,
# but right now it is just mainline code (so there is a
# bit of a race to copy the N bytes to/from the FIFO).

import copico_v2 as board
import micropython
import network
import rp2
import time
import wifi
import gc

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

DREQ_PIO0_TX_TX_SM = dma_defs.DREQ_PIO0_TX2  # Because TX_SM = 2
TXF_TX_SM_OFFSET = pio_defs.TXF2_OFFSET

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
    #NOPE# irq(0) # Tell the CPU that we have received a Control Write strobe                                          # type: ignore

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

def MainLoop(sm_mgr, sock):
    if not sm_mgr.sm_control_write:
        raise ValueError("Control Write State Machine has not been set up")
    print("MainLoop...")

    dma = None
    tx_data_bytearray = None
    GOOD = 2 # Since axiom uses 1 for TryAgain, we use 2 for Good.
    wrapped_good = WrapTxByteWithPindirs(GOOD)

    while True:
        # Busy wait for a control byte.
        while sm_mgr.sm_control_write.rx_fifo() == 0:
            pass

        print("=delay=0")
        control_byte = sm_mgr.sm_control_write.get()
        print(f'Control Byte Received: {control_byte}')

        if control_byte == 0:  # Stop Tx
            print("=21")

            def stop_sending_data(dma):
                print("=22")
                dma.active(False)
                print("=23")

            stop_sending_data(dma)
            print("=24")
            del dma
            # dma = None
            del tx_data_bytearray
            # tx_data_bytearray = None
            print("=25")
            sm_mgr.sm_status_read.put(wrapped_good)
            print("=26")

        elif control_byte <= 100:  # Rx
            n = control_byte
            a = bytearray(n)
            sm_mgr.sm_status_read.put(wrapped_good)

            for i in range(n):
                x = sm_mgr.sm_rx_write.get()
                sm_mgr.sm_status_read.put(wrapped_good)  # Allow coco to continue
                if i<10: print("rx[%d]: %d" % (i, x))
                a[i] = x

            #:# for i in range(n):
            #:#     print("RX [%d] %d" % (i, a[i]))

            sock.send(bytes(a))

        elif control_byte <= 200:  # Tx
            num_bytes_wanted = control_byte - 100
            tx_data_bytearray = bytearray(0)
            num_bytes_got = 0
            try:
                while num_bytes_got < num_bytes_wanted:
                    bytes_got = sock.recv(num_bytes_wanted - num_bytes_got)
                    print('TCP bytes_got len %d' % len(bytes_got))
                    for b in bytes_got:
                        tx_data_bytearray.append(255)  # Little End appends first
                        tx_data_bytearray.append(b)
                        tx_data_bytearray.append(0)
                        tx_data_bytearray.append(0)
                    num_bytes_got += len(bytes_got)
            except:
                 sm_mgr.sm_status_read.put(WrapTxByteWithPindirs(211)) # os9 E$EOF = 211 = $D3
                 raise

            # Start DMA

            def start_sending_dma(dma, tx_data_bytearray):
                dma_ctrl_val = dma.pack_ctrl(
                    default=0, # set all fields to 0 by default
                    size=dma_defs.SIZE_4BYTES, # send a word at a time
                    bswap=0, # Don't swap the byte order
                    enable=1, # enable this DMA channel
                    inc_read=1, # increment the read address
                    inc_write=0, # keep the same write address each time rather than incrementing
                    irq_quiet=1, # don't generate an IRQ when the transfer is complete
                    treq_sel=DREQ_PIO0_TX_TX_SM) # wait for Data Read state machine TX FIFO to have space before sending

                print('dma_ctrl_val = %s' % repr(dma_ctrl_val))
                dma.config(read=tx_data_bytearray, # where to read from
                        #write=sm_data_read, # (Specifying the state machine here doesn't work until the next version of uPy)
                        write=pio_defs.PIO0_BASE + TXF_TX_SM_OFFSET, # write to the Data Read state machine's TX FIFO
                        count=len(tx_data_bytearray) // 4, # number of words to transfer
                        ctrl=dma_ctrl_val,
                        trigger=True) # start the DMA transfer now
                print('dma configged')

            print('=1')
            dma = rp2.DMA()
            print('=2')
            start_sending_dma(dma, tx_data_bytearray)

            # Allow client to start reading the tx data
            print('=3')
            sm_mgr.sm_status_read.put(wrapped_good)
            print('=4')

            # Consume weird extra control byte
            while sm_mgr.sm_control_write.rx_fifo() == 0:
                pass

            control_byte = sm_mgr.sm_control_write.get()
            print(f'Bogus Control Byte: {control_byte}')

        else:
            print("Undefined control byte: %d" % control_byte)

        # gc.collect()
        # print('gc complete')


if __name__ == "__main__":
    # allow interrupts to throw errors
    micropython.alloc_emergency_exception_buf(200)

    print('Define Board Pins')
    pins = board.BoardPins()

    pins.led.value(1)   # LED on
    #pins.halt.value(0)  # 0 = Don't HALT; run CoCo
    #pins.slenb.value(0) # 0 = Don't assert SLENB; CoCo device select works normally
    pins.drive_data_bus.value(1)   # 1 = IN to PicoW from CoCo

    try:
        print('==> Create Wifi Manager')
        with wifi.WifiConnectionManager() as wifi_mgr:

            with wifi.SocketWithContextManager() as sock:
                host, port = 'pizga.net', 2327
                host, port = '192.168.86.235', 2321
                addr_tuple = wifi.get_address_tuple(host, port)

                print(f'Connecting to {addr_tuple}')
                sock.connect(addr_tuple)
                del addr_tuple

                print('==> Create PIO Manager')

                with PIOStateMachineManager() as sm_mgr:
                    print('==> Starting State Machines')
                    sm_mgr.start_state_machines()

                    print('==> Calling MainLoop')
                    MainLoop(sm_mgr, sock)


    finally:
        pins.led.value(0)
        del pins

        gc.collect()
        print('gc complete')
