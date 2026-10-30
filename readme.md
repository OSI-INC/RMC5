# Reduced Instruction Set Microcontroller, Five-Bit Opcodes (RMC5)

The RMC5 is an open-source eight-bit microcontroller with five-bit 
opcodes. The length of the instruction and process memory addresses
are the same and equal to the length of the operand carried with every
instruction, so that the instructions consists of a five-bit operand
and operand that could be an instruction address or a process adddress.
Our intention is for the instruction and process memories to be compact:
perhaps 128 locations each, and made with distributed memory so that
the compiler can optimise the combined processor and program by looking
at the instructions contained in the program. Using only a subset of the
instructions will result in a smaller compiled processor and program
memory.

The RMC5 is written in VHDL. Our first version is RMC5V1. Other versions
may follow. This repository contains a demonstration project in which
we implement an RMC5V1 for the purpose of compiling for size and speed
assessment. The demonstration project is a Lattice Diamond project. By
default, it uses the LCMXO2-256ZE device and connects all the RMC5
instruction and process signals to pins on the device so that all we
have in the device is the processor core itself.