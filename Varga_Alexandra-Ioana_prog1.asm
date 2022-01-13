
.data                  # directive indicating start of the data segment
.align  2              # set data alignment to 4 bytes

AddressLoc:                 # label - name of the memory block
.word  0, 0x14, 0x3c, 0x64, 0   # Location of Input Addresses for InpImgA, InpImgB, outputImg, NoofPixels

InpImgA:                 # label - name of the memory block
.word  0x5350412e, 2, 3, 0x11223300, 0x2200aaff,0x00ffff00,0x03565654,0x1b459748,0xecf39baa,0   # InputImg A values

InpImgB:                 # label - name of the memory block
.word  0x5350412e, 2, 3, 0x11223300, 0x2200aaff,0x00ffff00,0x03565654,0x1b459748,0xecf39baa,0   # InputImg B values

.text                 # beginning of the text segment (or code segment)

MAIN:
  lw a0, 0x4(zero)
  lw a1, 0x8(zero)
  lw a2, 0xc(zero)
  addi s11,zero,0x10
  jal PROGS         	# rutine call
  jal ENDPROGRAM	# End Program
 
PROGS:
add s6,ra,zero
#Load Pixel Dimensions
lw a3, 0(a0)
lw a4, 4(a0)
lw a5, 8(a0)
sw a3, 0(a2)
sw a4, 4(a2)
sw a5, 8(a2)

#multiply algorithm
addi t1,zero,0
addi s10,zero,1
LOOP1:
	slt	s8,t1,a4
	beq	s8,zero,LOOP1END
	addi t2,zero,0
	LOOP2:
		slt s8,t2,a5
		beq s8,zero,LOOP2END
		addi s9,s9,1
		addi t2,t2,1
		jal	LOOP2
		LOOP2END:
			addi t1,t1,1
			jal	LOOP1
LOOP1END:
add s5,zero,s9
START:
#Check All Pixels are merged
beq s9,zero,ENDRETURN


#Load Pixel
lw t1,12(a0)
lw t2, 12(a1)
addi t0,zero,255 #000000FF #adduqb t5,t1,t2

addi t6,zero,8
addi s7,zero,0xFF #000000FF
sll s7,s7,t6
addi s7,s7,0xFF #0000FFFF
sll s7,s7,t6
addi s7,s7,0xFF #00FFFFFF
and t5,t5,s7
lui s7,0xFF000 #FF000000
add t5,t5,s7

#store the pixel in output
sw t5,12(a2)

#Offset for next iteration
addi a0,a0,0x4
addi a1,a1,0x4
addi a2,a2,0x4
sub s9,s9,s10
j START

ENDRETURN:
#Store No of pixels at 00000010
addi a0,s5,0          # copy t4 (maximum) to a0 (a0 needs to hold the result)
sw a0, 0(s11)
add ra s6,zero
ret                   # return from the routine

ENDPROGRAM:
