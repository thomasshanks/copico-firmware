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
# netio: Respond to NETIO Control Port writes and Data Port reads
#
# ----------------------------------------------------------------------------
#
# This program is a proof of concept for the CoPiCo project. It demonstrates
# how to use the RP2040's PIO state machines to respond to writes to the
# Control Port and reads from the Data Port. The Control Port is used to start
# and stop sending data to the CoCo, while the Data Port is used to send data
# back to the CoCo in response to reads.
#
# Control Port Operation 0x01: Start sending data to CoCo
#
#   Writing 0x01 to the Control Port at $FF7A starts the PIO program that
#   sends bytes back to the CoCo in response to Data Port reads. Each read
#   from the Data Port at $FF7B will get the next byte in `BYTES_TO_SEND`
#   buffer until that buffer is exhausted. The PIO program will then no longer
#   respond to Data Port reads until the next write of 0x01 to the Control
#   Port.
#
# Control Port Operation 0x00: Stop sending data to CoCo
#
#   Writing 0x00 to the Control Port at $FF7A stops the PIO program that sends
#   bytes back to the CoCo in response to Data Port reads. The PIO program
#   will no longer respond to Data Port reads until the next write of 0x01 to
#   the Control Port.
#
#
# NOTE: Currently only supports the Aardvark/V1 board!
#

#from __future__ import annotations # Not supported on MicroPython

#import atexit # Not supported on MicroPython
import time

import micropython
import rp2
import _thread

import copico_v2 as board
import wifi

#from abc import ABC, abstractmethod # Not supported on MicroPython

# A special constant that is assumed to be True by 3rd party static type checkers. It is False at runtime.
TYPE_CHECKING = False

if TYPE_CHECKING:
    from typing import Optional

import dma_defs
import pio_defs

OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH

NETIO_PIO = 0
CONTROL_WRITE_SM = 0
DATA_READ_SM = 1

DATA_BUS_WIDTH = 8

#BYTES_TO_SEND = b"ALL wiznet AND NO pico MAKES\rcoco A DULL computer!\r"
BYTES_TO_SEND = b'SO RAISE YOUR JOYSTICKS, RAISE YOUR KEYBOARDS, LET CRTS ILLUMINATE THE WAY, WITH EVERY LINE, WITH EVERY BYTE, FOR COCO, WE PLEDGE ALLEGIANCE, THIS DAY!\r'

@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,   # Shift bits into the most significant end of the shift register
    autopush=True,                    # Automatically push the next word from input shift reg into FIFO
    push_thresh=8,                    # Push to the FIFO when 8 bits of the ISR have been produced
)
def on_control_write():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    #WRITE_CTRL_STROBE = board.WRITE_CONTROL_STROBE_PIN # GPIO strobe for Write Control (active high)
    CTRL_STROBE_N = 18 # board.CONTROL_STROBE_N_PIN # GPIO strobe for Control Register (active high)
    SM_IRQ = 4         # for State Machine communictaion

    wrap_target()                                                                                               # type: ignore

    #wait(1, gpio, WRITE_CTRL_STROBE) # Wait until WRITE_CTRL_STROBE is not being asserted (to prevent false positives)  # type: ignore
    #wait(0, gpio, WRITE_CTRL_STROBE) # Wait until WRITE_CTRL_STROBE is being asserted                                   # type: ignore
    wait(1, gpio, CTRL_STROBE_N) # Wait until CTRL_STROBE_N is not being asserted (to prevent false positives)  # type: ignore
    wait(0, gpio, CTRL_STROBE_N) # Wait until CTRL_STROBE_N is being asserted                                   # type: ignore

    jmp(pin, "read") # If the read/~write pin is high, jump to read handling                                    # type: ignore

    in_(pins, 8) # Send contents of data bus to input FIFO so they can be printed on the PicoW console          # type: ignore

    irq(0) # Tell the CPU that we have received a Control Write strobe                                          # type: ignore

    #irq(clear, SM_IRQ) # Tell the on_data_read state machine that it can start responding to Data Read strobes # type: ignore

    label("read") # TODO: Implement control register reading                                                    # type: ignore
    wrap()                                                                                                      # type: ignore

@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),    # Data bus pins will default to being active-high inputs
    #autopull=True,                    # Automatically pull the next word from FIFO into the output shift reg
    pull_thresh=24,                   # Pull from FIFO when 24 bits of the OSR have been consumed
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # Take bits from the least significant end of the shift register
    sideset_init=(OUT_HIGH),          # 15:drive_data_bus (1 = IN to PicoW from CoCo)
#   sideset_init=(OUT_HIGH, OUT_LOW), # 15:drive_data_bus (1 = IN to PicoW from CoCo); 16:trigger (1 = trigger oscilloscope capture)
)
def on_data_read():
    # Constants in PIO ASM must be (re)defined inside program; globals are not accessible
    #READ_DATA_STROBE = board.READ_DATA_STROBE_PIN # GPIO strobe for Read Data (active high)
    DATA_STROBE_N = 19 # board.DATA_STROBE_N_PIN # GPIO strobe for Data Register (active high)
    SM_IRQ = 4        # IRQ to wait on before responding to reads on the Data Port

    wrap_target()                                                                                               # type: ignore

    # Wait for a signal from the control port indicating that we can start responding to Data Read strobes
    #irq(block, SM_IRQ)                                                                                         # type: ignore

    pull(ifempty) # Get the next word from the FIFO and put it into the output shift register                   # type: ignore

    #wait(1, gpio, READ_DATA_STROBE) # Wait until READ_DATA_STROBE is not being asserted (to prevent false positives)  # type: ignore
    #wait(0, gpio, READ_DATA_STROBE) # Wait until READ_DATA_STROBE is being asserted                                   # type: ignore
    wait(1, gpio, DATA_STROBE_N) # Wait until DATA_STROBE_N is not being asserted (to prevent false positives)  # type: ignore
    wait(0, gpio, DATA_STROBE_N) # Wait until DATA_STROBE_N is being asserted                                   # type: ignore

    jmp(pin, "read") # If the read/~write pin is high, jump to read handling                                    # type: ignore
    jmp("end") # Otherwise, jump to write handling # TODO: Implement data register writing                      # type: ignore
    label("read")                                                                                               # type: ignore

    out(pindirs, 8) # Set the data bus pins to be outputs on the PicoW side                                     # type: ignore

    out(pins, 8).side(0b0)  # drive_data_bus=0 (OUT to CoCo)                                                    # type: ignore
    #            .side(0b10) # trigger=1; drive_data_bus=0 (OUT to CoCo)                                        # type: ignore
    #wait(1, gpio, READ_DATA_STROBE) # Wait until READ_DATA_STROBE is no longer asserted                        # type: ignore
    wait(1, gpio, DATA_STROBE_N) # Wait until DATA_STROBE_N is no longer asserted                               # type: ignore
    nop().side(0b1)  # drive_data_bus=1 (IN from CoCo)                                                          # type: ignore
    #    .side(0b01) # trigger=0; drive_data_bus=1 (IN from CoCo)                                               # type: ignore

    out(pindirs, 8) # Set the data bus pins to be inputs on the PicoW side                                      # type: ignore

    label("end")                                                                                                # type: ignore
    wrap()                                                                                                      # type: ignore

class DataProducer(): # ABC
    bytes_left: int

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.cleanup()

    def start_sending_data(self):
        raise NotImplementedError("Subclasses must implement this method")

    def stop_sending_data(self):
        raise NotImplementedError("Subclasses must implement this method")

    def cleanup(self):
        raise NotImplementedError("Subclasses must implement this method")

class PIOStateMachineManager:
    def __init__(self, data_producer: "Optional[DataProducer]" = None):
        self.data_producer = data_producer
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

        self.sm_control_write = rp2.StateMachine(
            (NETIO_PIO << 3) + CONTROL_WRITE_SM, # which state machine in pio0
            on_control_write,
            freq=125_000_000,
            in_base=board.FIRST_DATA_PIN,
            jmp_pin=board.READ_NOT_WRITE_PIN,
        )

        self.sm_data_read = rp2.StateMachine(
            (NETIO_PIO << 3) + DATA_READ_SM, # which state machine in pio0
            on_data_read,
            freq=125_000_000,
            out_base=board.FIRST_DATA_PIN,
            sideset_base=board.FIRST_SIDESET_PIN,
            jmp_pin=board.READ_NOT_WRITE_PIN,
        )

        def handler(sm):
            if not self.sm_control_write:
                raise ValueError("Control Write State Machine has not been set up")

            control_byte = self.sm_control_write.get()
            print(f'Control Byte Received: {hex(control_byte)}')
            if control_byte == 0x00:
                self.stop_sending_data()
            elif control_byte == 0x01:
                self.start_sending_data()

        # TODO: What are the `trigger=0|1` and `hard=True|False` options on `.irq(..)` for?
        self.sm_control_write.irq(handler=handler)

    def start_state_machines(self):
        if not self.sm_control_write or not self.sm_data_read:
            raise ValueError("State machines have not been set up")

        self.sm_control_write.active(True)
        self.sm_data_read.active(True)

    def start_sending_data(self):
        if self.data_producer:
            self.data_producer.start_sending_data()

        #self.sm_data_read.active(True)

    def stop_sending_data(self):
        if self.data_producer:
            self.data_producer.stop_sending_data()

        if self.sm_data_read and self.sm_data_read.active():
            #self.sm_data_read.active(False)

            # Empty the TX FIFO by throwing away the contents of the OSR five times
            for _ in range(5):
                self.sm_data_read.restart()

    def stop_state_machines(self):
        if self.sm_control_write:
            self.sm_control_write.active(False)
        if self.sm_data_read:
            self.sm_data_read.active(False)

    def cleanup(self):
        self.stop_sending_data()
        self.stop_state_machines()
        del self.sm_control_write
        del self.sm_data_read

        # Remove all existing PIO programs from PIO 0
        rp2.PIO(NETIO_PIO).remove_program()

        print('PIO freed')

class CPUDataProducer(DataProducer):
    def __init__(self, sm_data_read: Optional[rp2.StateMachine], bytes_to_send: bytes):
        self.thread_started = False
        self.should_exit = False
        self.should_send_data = False

        self.sm_data_read = sm_data_read
        self.bytes_to_send = bytes_to_send

        self.bytes_left = 0

    def start_sending_data(self):
        self.should_send_data = True

        if not self.thread_started:
            self.thread_started = True
            _thread.start_new_thread(self._senddata_task, (self.sm_data_read, self.bytes_to_send))

    def stop_sending_data(self):
        self.should_send_data = False

    def cleanup(self):
        self.stop_sending_data()
        self.should_exit = True

    def _senddata_task(self, sm_data_read: rp2.StateMachine, bytes_to_send: bytes):
        """Enqueue data into Data Read PIO State Machine's TX FIFO for sending to the CoCo."""
        buff_length = len(bytes_to_send)

        while not self.should_exit:
            for index, byte in enumerate(bytes_to_send):
                # Keep trying to send the next `byte` in `bytes_to_send` until we have
                # successfully sent it or we are asked to stop sending data
                while self.should_send_data:
                    self.bytes_left = buff_length - index

                    # Don't try to send the next byte unless the TX FIFO has
                    # room for it
                    if sm_data_read.tx_fifo() < 4:
                        # Wrap the byte to send with the pindirs values to
                        # configure before (0xFF) and after (0x00)
                        word_to_send = 0x0000FF | (byte << 8)
                        print(f'Queuing {hex(word_to_send)} ("{chr(byte)}" surrounded by the pindirs)')
                        sm_data_read.put(word_to_send)

                        # Go on to the next `byte` in `bytes_to_send`
                        break
                else: # if not self.should_send_data:
                    self.bytes_left = 0

                    # TODO: Wait on a _thread.lock() that indicates it is time
                    # to start sending data again instead of spinning in the
                    # `while True:` loop

                    # Exit the `for byte in bytes_to_send:` loop
                    break
            else: # if we've reached the end of the data...
                self.bytes_left = 0

                # ... then stop sending, but don't stop the state machine
                self.should_send_data = False

class DMADataProducer(DataProducer):
    def __init__(self, sm_data_read: "Optional[rp2.StateMachine]", bytes_to_send: bytes):
        self.sm_data_read = sm_data_read

        # Wrap each byte to send in the `pindirs` information, which the PIO will
        # use to set the direction of the GPIO pins that are connected to the data
        # bus
        self.tx_data_bytearray = bytearray().join(
            (0x0000_00FF | (byte << 8)).to_bytes(4, 'little') for byte in BYTES_TO_SEND)
        print(f'src_data[{len(self.tx_data_bytearray)}] = {self.tx_data_bytearray.hex()}')

        self.dma = rp2.DMA()

        #atexit.register(self.cleanup)

    @property
    def bytes_left(self) -> int:
        return self.dma.count # type: ignore

    def start_sending_data(self):
        # See https://github.com/Hack-a-Day/Vectorscope/blob/main/source/pixel_pusher.py for
        # an example of how to use the DMA to send data to a PIO state machine

        dma_ctrl_val = self.dma.pack_ctrl(
            default=0, # set all fields to 0 by default
            size=dma_defs.SIZE_4BYTES, # send a word at a time
            bswap=0, # Don't swap the byte order
            enable=1, # enable this DMA channel
            inc_read=1, # increment the read address
            inc_write=0, # keep the same write address each time rather than incrementing
            irq_quiet=1, # don't generate an IRQ when the transfer is complete
            treq_sel=dma_defs.DREQ_PIO0_TX1) # wait for Data Read state machine TX FIFO to have space before sending

        self.dma.config(read=self.tx_data_bytearray, # where to read from
                #write=sm_data_read, # (Specifying the state machine here doesn't work until the next version of uPy)
                write=pio_defs.PIO0_BASE + pio_defs.TXF1_OFFSET, # write to the Data Read state machine's TX FIFO
                count=len(self.tx_data_bytearray) // 4, # number of words to transfer
                ctrl=dma_ctrl_val,
                trigger=True) # start the DMA transfer now

    def stop_sending_data(self):
        self.dma.active(False)

    def cleanup(self):
        self.stop_sending_data()
        self.dma.close()
        del self.dma
        del self.tx_data_bytearray

        print('DMA freed')

if __name__ == "__main__":
    # allow interrupts to throw errors
    micropython.alloc_emergency_exception_buf(100)

    pins = board.BoardPins()

    pins.led.value(0)   # LED off
    #pins.halt.value(0)  # 0 = Don't HALT; run CoCo
    #pins.slenb.value(0) # 0 = Don't assert SLENB; CoCo device select works normally
    pins.drive_data_bus.value(1)   # 1 = IN to PicoW from CoCo

    #bytes_to_send = BYTES_TO_SEND

    with wifi.WifiConnectionManager() as wifi_mgr:
        host, port = wifi.load_host_and_port_from_file()
        data_rcvd = wifi.read_url(host, port)
        del host; del port

    print(data_rcvd.decode('ascii'))
    bytes_to_send = data_rcvd

    try:
        with PIOStateMachineManager() as sm_mgr:
            # Both of these currently work, but the DMA version is more efficient
            with CPUDataProducer(sm_mgr.sm_data_read, bytes_to_send) as data_producer:
            #with DMADataProducer(sm_mgr.sm_data_read, bytes_to_send) as data_producer:
                sm_mgr.data_producer = data_producer

                sm_mgr.start_state_machines()

                # Blink LED at 1 Hz
                pins.led.value(1)

                while True:
                    pins.led.value(1)
                    time.sleep(0.2)
                    pins.led.value(0)
                    time.sleep(0.8)
                    if data_producer.bytes_left > 0:
                        print(f'DATA REMAINING: {data_producer.bytes_left}')
                    if sm_mgr.sm_data_read and sm_mgr.sm_data_read.tx_fifo():
                        print(f'TX FIFO: {sm_mgr.sm_data_read.tx_fifo()}')
    finally:
        pins.led.value(0)
        del pins

        import gc; gc.collect()
        print('gc complete')
