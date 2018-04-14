RISC-V bare metal Program
=========================

This directory contains code to build a very small and rudimentary bare metal application.


Building the Application
------------------------

Assuming riscv64-unknown-elf-gcc and riscv64-unknown-elf-as are available in your PATH, you can just hit:
```
make
```
This will build an application called 'bare_metal' in the folder tests/test-progs/bare_metal/bin/riscv


Source Code
-----------

Files in this directory:
 * link.ld: A small linker script.
 * startup.S: This file contains the startup code. The reset vector just jumps to the symbol 'startup', which lies in the text section. There all gp registers are zeroed. Afterwards the stack is initialised and control is given to the main function.
 * main.c: Main function, that traps in a while loop.  
