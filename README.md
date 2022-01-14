# MIPS-32-BIT-Merge-Pixels-Image

---------------------------------------------------------------------------------------------------------------------
This project consists of designing the 32-bit single cycle mips processor in the Verilog programming language. 
The processor can execute a variety of instructions with which a complex program can be implemented. 
I have produce in RISC-V Assembly Language a solution that can combine pixels of two given images, 
and it being transformed into machine code will run on the presented processor.
---------------------------------------------------------------------------------------------------------------------

Design a simple 32-bit processor connected to a separate instruction and data memory. 
The processor has to implement instructions given in the table bellow. 
Suppose that the processor starts the execution from the beginning of instruction memory (0x00000000).

Write a program called S in the RISC-V assembly language that merges two images into one image. We will use image format described later. 
Program calls a subroutine merge() with the following C-language prototype:

int merge(int *inputImgA, int *inputImgB, int *outputImg);

This subroutine should return the number of pixels of the output image. Use the RISC-V calling convention.

Suppose that the input addresses for the subroutine are stored in the data memory at following addresses:

0x00000004: inputImgA
0x00000008: inputImgB
0x0000000C: outputImg
For instance, at address 0x00000004 the address of inputImgA is stored (where the image begins).

After returning from the subroutine, program should write the returned result (i.e. the number of pixels of the output image) into the data memory at address of 0x00000010.

Translate program S from the RISC-V assembly language into the machine code of your CPU design.

