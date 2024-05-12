# Proof of Concept MicroPython "Firmware"for the CoPiCo Board's RP2040 Microcontroller

The CoPiCo is a cartridge for the Tandy Color Computer that adds WiFi to and
serves as a boot ROM for the system. It is based on the Raspberry Pi Pico W
microcontroller dev board, which includes onboard WiFi.

![copico_pcb_rev0.2_front](https://github.com/thomasshanks/copico-board/blob/main/assets/copico_rev0.2_front.png)

This proof-of-concept "firmware" runs in MicroPython on the CoPiCo. It connects
to a predetermined TCP port and streams the next byte to the Tandy Color
Computer whenever any address in the range 0xFF68-0xFF6B or 0xFF78-0xFF7B is
read. Writing to any address in this range will reconnect to the TCP stream and
restart the process.

## WiFi Connectivity

The real firmware will behave more like a SPI WiFi module or the WizNet chip
(or perhaps both, depending on mode). It will, of course, also allow listing
and connecting to the desired WiFi network.

## ROM Emulation

CoPiCo will also support boot ROM emulation (initially just for a 1 KB ROM due
to limitations of first revision of the CoPiCo PCB). Henry Strickland
(github.com/strickyak) plans to use this feature to load a version of his
"axiom" network boot ROM (see
github.com/strickyak/frobio/tree/main/frob3/booting).

## Other Future Capabilities

Future capabilities for the CoPiCo may include:
- VGA or HDMI output based on the captured contents of video RAM
- Logic capture / bus trace
- Remote debug, with breakpoints and watchpoints

## Stay Tuned to This Channel

Please star this repo and/or watch this space for updates as development
continues!

Thank you for being a part of the Retrocomputing community!


Thomas Shanks (copico@tshanks.org)
