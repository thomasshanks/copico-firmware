#
# MIT License
#
# Copyright 2023 Elliot Williams
# Copyright 2023 Voja Antonic
# Copyright 2023 Al Williams
# Copyright 2023 Tom Nardi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions: 
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software. 
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
#
# Retrieved from:
#
# https://github.com/Hack-a-Day/Vectorscope/blob/main/source/pio_defs.py 
#

PIO0_BASE = const(0x50200000)
PIO1_BASE = const(0x50300000)

CLKDIV_RESTART_BIT = const(8)
SM_ENABLE_BIT      = const(0) 

FSTAT_OFFSET  = const(0x004)
FDEBUG_OFFSET = const(0x008)

TXF0_OFFSET   = const(0x010)
TXF1_OFFSET   = const(0x014)
TXF2_OFFSET   = const(0x018)
TXF3_OFFSET   = const(0x01C)

RXF0_OFFSET   = const(0x020)
RXF1_OFFSET   = const(0x024)
RXF2_OFFSET   = const(0x028)
RXF3_OFFSET   = const(0x02C)

## Direct write-only access to instruction memory.
INSTR_MEM0 = const(0x048)
INSTR_MEM1 = const(0x04C)
# etc.
INSTR_MEM31 = const(0x0C4)

SM0_CLKDIV = const(0x0C8)
SM1_CLKDIV = const(0x0E0)
SM2_CLKDIV = const(0x0F8)
SM3_CLKDIV = const(0x110)
## Frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256)
