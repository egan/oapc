;====================================================================;
;--------------------------------------------------------------------;
; INTERNAL DATA SETUP: Stack bottom must be set to 0x70 in main      ;
;                      program to prevent conflicts.                 ;
;--------------------------------------------------------------------;
;====================================================================;

; System flags.
XGO	def 21H.0		; Motion start/stop flag.
INPOS def 21H.1		; Servo lock flag.
DATSAV def 21H.3
MCDIR def 21H.4		; Motion direction flag.
MSIGN0 def 21H.5	; Multiplicand sign flag.
MSIGN1 def 21H.6	; Multiplier sign flag.
MSIGNALL def 21H.7	; Product sign flag.

; Timing counter value (16-bit).
CNTL epz 23H
CNTH epz 24H

; XXX: Buffer address value (16-bit).
BUFF_ADRL def 4AH
BUFF_ADRH def 4BH

; Velocity command value VELX (16-bit).
VELXH epz 4CH
VELXL epz 4DH

; Position increment/decrement DXC (16-bit).
DXCH epz 4EH
DXCL epz 4FH

; Up/Down counter value XC (16-bit).
XCH epz 50H
XCL epz 51H

; Position interpolation increment DFX (16-bit).
DFXH epz 53H
DFXL epz 52H

; Position following error PEX (16-bit).
PEXH epz 54H
PEXL epz 55H

; Residual distance to end position FX (32-bit).
FX1 epz 56H
FX2 epz 57H
FX3 epz 58H
FX4 epz 59H

; Proportional gains.
KP epz 5AH		; During motion.
KPINP epz 5BH	; Servo locked (in position).

; Distance motion command per sampling period DX (16-bit).
DXH epz 5EH
DXL epz 5FH

; Absolute position counter value ABSX (32-bit).
ABSX1 epz 60H
ABSX2 epz 61H
ABSX3 epz 62H
ABSX4 epz 63H

; XXX: Unknown use.
RDABSEN epz 64H	; Enable updating software absolute position counter.
INTCNT epz 65h

; Temporary variable (16-bit) for position inc/decrement calculation.
DABSH epz 66H
DABSL epz 67H

; Digital-Analog conversion and control addresses.
_DA1_LOW equ 0FE18H
_DA1_HIGH equ 0FE19H
_DA_CNVT equ 0FE1FH

; Up/Down counter countrol addresses.
_UDCNT1_LOW equ 0FF05H
_UDCNT1_HIGH equ 0FF06H

;====================================================================;
;--------------------------------------------------------------------;
;         INITIALIZATION: Setup on initial timer interrupt.          ;
;--------------------------------------------------------------------;
;====================================================================;
org 0x9F00
push psw			; Save main PSW to stack.
orl psw, #0x18		; Use register bank 3 from now on.
push acc			; Save main arithmetic registers to stack.
push b
push dpl			; Save main data pointer registers to stack.
push dph
mov dptr, #0xFA1B	; Set Timer1 interrupt vector address (0xFA1B).
mov a, #0x02		; Setup interrupt vector: LJMP 0x9F50.
movx @dptr, a
inc dptr
mov a, #0x9F
movx @dptr, a
inc dptr
mov a, #0x50
movx @dptr, a

; At 20MHz, 1ms => 0xF97D, 3ms => 0xEC78, 10ms => 0xBEE5
mov tmod, #0x10		; Setup Timer1 as 16-bit timer.
mov tl1, #0xE5		; Setup Timer1 overflow period (10ms).
mov th1, #0xBE

lcall ClearCounter	; Clear the U/D counter.
mov a, #0x00		; Start DAC with value 0x8000.
mov b, #0x80
lcall DAOut
lcall INITVAR		; Setup initial values of variables.
setb TR1			; Start Timer1.
orl ie, #0x88		; Enable interrupts from Timer1 overflow.

pop dph				; Restore stacked registers.
pop dpl
pop b
pop acc
pop psw
ret					; Done.


;====================================================================;
;--------------------------------------------------------------------;
;          REALTIME SYSTEM: Main code for timer interrupt.           ;
;--------------------------------------------------------------------;
;====================================================================;
org 0x9F50
push b			; Save main arithmetic registers to stack.
push acc
orl psw, #0x18	; Use register bank 3 from now on.
mov tl1, #0xE5	; Setup Timer 1 overflow period (10ms).
mov th1, #0xBE
push dpl		; Save data pointer registers to stack.
push dph

;mov dptr, #0xFF00
;mov a, #0xFF
;movx @dptr, a

mov dph, CNTH	; Increment timing counter.
mov dpl, CNTL
inc dptr
mov CNTH, dph
mov CNTL, dpl

; Call each of the modules in turn.
lcall ReadUDCounter		; Module 1.
lcall PosDiffCAL		; Module 2.
lcall PEXUpdate			; Module 3.
lcall INPOSJudge		; Module 4.
lcall PPCAL				; Module 5.
lcall DAOUTVEL			; Module 6.
lcall INTPAndFXCheck	; Modules 7 and 8.

;mov dptr, #0xFF00
;mov a, #0x00
;movx @dptr, a

done:
pop dph			; Restore staced registers.
pop dpl
pop acc
pop b
pop psw			; XXX: Was PSW ever pushed to stack?
reti

;====================================================================
; subroutine INITVAR
; Set Up Initial Values of Variables
;====================================================================
INITVAR:
mov ABSX4, #0x00	; Start absolute position ABSX at 0.
mov ABSX3, #0x00
mov ABSX2, #0x00
mov ABSX1, #0x00
mov PEXH, #0x00		; Start position error PEX at 0.
mov PEXL, #0x00
mov DXH, #0x00		; Start distance motion commant at 0.
mov DXL, #0x00
mov XCH, #0x00		; Start UDC value at 0.
mov XCL, #0x00
setb INPOS			; Lock servo in position.
clr DATSAV
clr XGO				; Stop motion control.
mov INTCNT, #0x00
mov RDABSEN, #0x00
mov DABSH, #0x00	; Start temporary distance variables at 0.
mov DABSL, #0x00
mov CNTH, #0x00		; Start the timer at 0.
mov CNTL, #0x00		;
ret

;====================================================================
; subroutine ClearCounter
; Clear U/D Counter1
;====================================================================
ClearCounter:
mov dptr, #_UDCNT1_HIGH	; Send reset command to UDC.
mov a, #0x00
movx @dptr, a
ret

;====================================================================
; subroutine DAOut
; Output to Digital-Analog Converter
;
; inputs: b, acc = D
;====================================================================
DAOut:
mov dptr, #_DA1_LOW		; Push D low byte to DAC.
movx @dptr, a
nop						; Wait two computation cycles.
nop
mov a, b				; Push D high byte to DAC
mov dptr, #_DA1_HIGH
movx @dptr,a
nop						; Wait two computation cycles.
nop
mov dptr, #_DA_CNVT		; Start D/A conversion.
movx @dptr, a
ret

;====================================================================
; subroutine ReadUDCounter
; 1. Read Up/Down counter.
;
; output: XCH, XCL = UDC counter value (16-bit)
;====================================================================
ReadUDCounter:
mov r3, XCH				; Store value at last period.
mov r2, XCL
mov dptr, #_UDCNT1_LOW	; Send start read enable command to UDC.
mov a, #0x00
movx @dptr, a
mov dptr, #_UDCNT1_LOW	; Read low byte XCL.
movx a, @dptr
mov XCL, a
mov dptr, #_UDCNT1_HIGH	; Read high byte XCH.
movx a, @dptr
mov XCH, a
ret

;====================================================================
; subroutine PosDiffCAL
; 2. Motion inc/decrement calculation and absolute counter update.
;
; inputs: XCH, XCL = XC; ABSX4, ABSX3, ABSX2, ABSX1 = ABSX
;
; output: DXCH, DXCL = DXC_i = XC_i - XC_{i-1}
;         ABSX4, ABSX3, ABSX2, ABSX1 = ABSX_i = ABSX_{i-1} + DXC_i
;
; alters: DABSH, DABSL, register bank 3
;====================================================================
PosDiffCAL:
; Calculate position encoder inc/decrement.
mov r1, XCH			; Subtract last XC (loaded into registers by
mov r0, XCL			; ReadUDCounter) from current XC.
lcall CSUB16
mov DXCH,r1			; Store result in DXC.
mov DXCL,r0
; Calculate absolute position inc/decrement.
mov r3, DABSH		; Add DXC to DABS.
mov r2, DABSL
lcall CADD16
mov DABSH, r1		; And store result in DABS.
mov DABSL, r0
mov a, RDABSEN		; Is RDABSEN 0?
xrl a, #0x01
jnz ABXUpdate		; If RDABSEN == 0, continue.
ret					; Else return.

ABXUpdate:
mov r1, DABSH		; Cast DABS to 24-bit format.
mov r0, DABSL
lcall C16toC24
mov r7, ABSX4		; Add DABS to ABSX.
mov r6, ABSX3
mov r5, ABSX2
mov r4, ABSX1
lcall CADD24
mov ABSX4, r3		; And store result in ABSX.
mov ABSX3, r2
mov ABSX2, r1
mov ABSX1, r0
mov DABSH, #0x00	; Reset DABS.
mov DABSL, #0x00
ret

;***********************************************************************
;3. Position error update
;
;Input:
; DXCH: Position difference at adjacent sampling time HIGH byte
; DXCL: Position difference at adjacent sampling time LOW byte
; DXH: Distance motion command per sampling period HIGH byte
; DXL: Distance motion command per sampling period LOW byte
; PEXH: Position following error at last sampling time HIGH byte
; PEXL: Position following error at last sampling time LOW byte
;
;Output:
; PEXH: Position following error at this sampling time HIGH byte
; PEXL: Position following error at this sampling time LOW byte
;
;***********************************************************************
PEXUpdate:
mov r1,PEXH ;Last PEX
mov r0,PEXL ;last PEX
mov r3,DXCH
mov r2,DXCL ;PEXi=PEX(i-1)-DXC+(-)DX
lcall CSUB16 ;
mov r3,DXH
mov r2,DXL ;
jnb MCDIR ,PlusDir ;
lcall CSUB16 ;-dir
sjmp OutPEX
PlusDir:	;+dir
lcall CADD16
OutPEX:
mov PEXH,r1
mov PEXL,r0
ret

;***********************************************************************
;4. In-position Discrimination
;
;Input:
; XGO: Motion start flag at current sampling time
; INPOS: Servo locked flag at last sampling time
;
;Output:
; INPOS: Servo locked flag at this sampling time
;
;***********************************************************************
INPOSJudge:
jnb XGO, MCStop
clr INPOS
ret
MCStop:
jnb INPOS, SetINPOS
ret
SetINPOS:
setb INPOS
ret

;***********************************************************************
;5. Proportional control algorithm calculation
;
;Input:
; PEXH: Position following error at this sampling time HIGH byte
; PEXL: Position following error at this sampling time LOW byte
; KP: Proportion control gain
; KPINP: Servo locked gain
; INPOS: Servo locked flag
;Output:
; VELXH: Velocity command value HIGH byte
; VELXL: Velocity command value LOW byte
;
;***********************************************************************
PPCAL:
mov a, KP
jnb INPOS, CALPP
mov a, KPINP ; INPOS gain
CALPP:
mov r2, a
mov r3, #00H ;
mov r1, PEXH
mov r0, PEXL
lcall mul16
mov VELXL, r0
mov VELXH, r1
ret

;***********************************************************************
;6. D-to-A Conversion for velocity command output
;
;Input:
; VELXH: Velocity command value HIGH byte
; VELXL: Velocity command value LOW byte
;Output:
; VELDA: Digital Velocity command value
;
;***********************************************************************
DAOUTVEL:
jb MSIGNALL, VELNEG
mov r0,#0FFh	;VELX is Positive
mov r1,#07Fh
mov r2,VELXL
mov r3,VELXH
lcall CADD16
sjmp WROutput
VELNEG:	;VELX is Negative
mov r0,#0FFh
mov r1,#07Fh
mov r2,VELXL
mov r3,VELXH
lcall CSUB16
WROutput: ; write to D/A converter
mov a,r0
mov b,r1
lcall DAOut
ret

;***********************************************************************
;7. Interpolation command for next interrupt and Final Position update
;
;Input:
; DFXH: Distance to be moved per sampling period HIGH byte
; DFXL: Distance to be moved per sampling period LOW byte
; FX4: Distance to be moved FOURTH byte
; FX3: Distance to be moved THIRD byte
; FX2: Distance to be moved SECOND byte
; FX1: Distance to be moved FIRST byte
; INPOS: Servo locked flag
;Output:
; DXH: Distance motion command per sampling period HIGH byte
; DXL: Distance motion command per sampling period HIGH byte
; XGO: Motion start/stop flag;
; FX4: Next distance to be moved FOURTH byte
; FX3: Next distance to be moved THIRD byte
; FX2: Next distance to be moved SECOND byte
; FX1: Next distance to be moved FIRST byte
;
;***********************************************************************
INTPAndFXCheck:
jnb XGO, StopINTP
mov r1,DFXH
mov r0,DFXL
lcall C16toC24 ;r0,r1,r2,r3
mov r7,FX4
mov r6,FX3
mov r5,FX2
mov r4,FX1
lcall CSUB24 ;
mov FX4,r3
mov FX3,r2
mov FX2,r1
mov FX1,r0
sjmp FXCheckXGO

;***********************************************************************
;8. Interpolation end Discrimination and update distance¨Cto-go at final period
;
;Input:
; FX4: Distance to be moved FOURTH byte
; FX3: Distance to be moved THIRD byte
; FX2: Distance to be moved SECOND byte
; FX1: Distance to be moved FIRST byte
;Output:
; DXH: Distance motion command per sampling period HIGH byte
; DXL: Distance motion command per sampling period HIGH byte
; XGO: Motion start/stop flag;
;
;***********************************************************************

FXCheckXGO:
mov a,FX3
jb acc.7, CLRXGO ;FX is negative
mov DXH, DFXH
mov DXL, DFXL
ret

CLRXGO:
mov DXH,r5
mov DXL,r4
CLR XGO
ret
StopINTP:
mov DXH,#000h
mov DXL,#000h
ret

;====================================================================;
;--------------------------------------------------------------------;
;       ARITHMETIC ROUTINES: 16-, 24-, 32-Bit 2's Complement         ;
;--------------------------------------------------------------------;
;====================================================================;

;====================================================================
; subroutine CADD16
; 16-Bit Signed (2's Complement) Addition
;
; inputs: r1, r0 = X
;         r3, r2 = Y
;
; output: r1, r0 = signed sum S = X + Y
;
; alters: acc, C, OV, register bank 3
;====================================================================
CADD16:
orl psw, #0x18
mov a, r0	; Add low bytes together.
add a, r2
mov r0, a	; Save result in S low byte.
mov a, r1	; Add high bytes together with carry.
addc a, r3
mov r1, a	; Save result in S high byte.
clr C
ret

;====================================================================
; subroutine CSUB16
; 16-Bit Signed (2's Complement) Subtraction
;
; inputs: r1, r0 = X
;         r3, r2 = Y
;
; output: r1, r0 = signed difference D = X - Y
;
; alters: acc, C, OV, register bank 3
;====================================================================
CSUB16:
orl psw, #0x18
clr C
mov a, r0	; Subtract low bytes.
subb a, r2
mov r0, a	; Save result in D low byte.
mov a, r1	; Subtract high bytes with borrow.
subb a, r3
mov r1, a	; Save result in D high byte.
clr C
ret

;====================================================================
; subroutine MUL16
; 16-Bit Signed (2's Complement) Multiplication to 32-Bit Product
; with Sign Flag
;
; inputs: r1, r0 = X
;         r3, r2 = Y
;
; output: r3, r2, r1, r0 = product magnitude P = |X x Y|
;         MSIGNALL = sign(P): 1 if negative
;
; calls: UMUL16, Cr0r1, Cr2r3, Mr0r3
;
; alters: acc, C, OV, MSIGN0, MSIGN1, MSIGNALL, register bank 3
;====================================================================
MUL16:
orl psw, #0x18
clr MSIGNALL	; Reset product sign flag.
lcall Cr0r1		; Find magnitude and sign of multiplicand X.
lcall Cr2r3		; Find magnitude and sign of multiplier Y.
lcall UMUL16	; Perform unsigned multiplication of |X| x |Y|.
lcall Mr0r3		; Find sign of product P.
;lcall FRound
ret

;====================================================================
; subroutine UMUL16
; 16-Bit Unsigned Multiplication to 32-Bit Product
;
; inputs: r1, r0 = |X|
;         r3, r2 = |Y|
;
; output: r3, r2, r1, r0 = product P = |X| x |Y|
;
; alters: acc, C, OV, register bank 3
;====================================================================
UMUL16:
orl psw, #0x18
push b		; Save extra registers we will use to stack.
push dpl
mov a, r0	; Multiply XL x YL.
mov b, r2
mul ab
push acc	; Stack result low byte.
push b		; Stack result high byte.
mov a, r0	; Multiply XL x YH.
mov b, r3
mul ab
pop 18H		; Pop to R0.
add a, r0	; Add result low byte to first result high byte.
mov r0, a
clr a
addc a, b	; Add carry to result high byte.
mov dpl, a	; Store carried result high byte in DPL.
mov a, r2	; Multiple XH x YL.
mov b, r1
mul ab
add a, r0	; Add result low byte to R0.
mov r0, a
mov a, dpl
addc a, b	; Add result high byte to DPL.
mov dpl, a
clr a
addc a, #0	; Carry and save to R4.
mov r4, acc
mov a, r3	; Multiply XH x Yh.
mov b, r1
mul ab
add a, dpl	; Add result low byte to DPL.
mov r2, a	; Save product 3rd byte.
mov acc, r4 ; Retrieve carry and add to result high byte.
addc a, b
mov r3, a	; Save product high byte.
mov r1, 18H	; Save product second byte from R0.
pop 18H		; Retrieve product low byte.
pop dpl		; Restore registers.
pop b
ret

;===================================================================
; subroutine Cr0r1
; 16-Bit 2's Complement Magnitude and Sign
;
; inputs: r1, r0 = X
;
; output: r1, r0 = |X|
;         MSIGN1 = sign(X): 1 if negative
;
; alters: acc, C, MSIGN1, register bank 3
;===================================================================
Cr0r1:
orl psw, #0x18
mov a, r1
jb acc.7, c0a	; Negative if high byte bit 7 is 1.
clr MSIGN1		; Clear sign bit if positive.
ret				; Done.

c0a:
setb MSIGN1		; Set sign bit because negative.
mov a, r0		; 1's complement low byte.
cpl a
add a, #1		; And add 1 to do 2's complement.
mov r0, a		; Save result low byte.
mov a, r1		; 2's complement high byte.
cpl a
addc a, #0		; With carry from low byte.
mov r1, a		; Save result high byte.
ret

;====================================================================
; subroutine Cr2r3
; 16-Bit 2's Complement Magnitude and Sign
;
; inputs: r3, r2 = Y
;
; output: r3, r2 = |Y|
;         MSIGN0 = sign(X): 1 if negative
;
; alters: acc, C, MSIGN0, register bank 3
;====================================================================
Cr2r3:
orl psw, #0x18
mov a, r3
jb acc.7, c1a	; Negative if high byte bit 7 is 1.
clr MSIGN0		; Clear sign bit if positive.
ret				; Done.

c1a:
setb MSIGN0		; Set sign bit because positive.
mov a, r2		; 1's complement low byte.
cpl a
add a, #1		; And add 1 to do 2's complement.
mov r2, a		; Save result low byte.
mov a, r3		; 2's complement high byte.
cpl a
addc a, #0		; With carry from low byte.
mov r3, a		; Save result high byte.
ret

;====================================================================
; subroutine C2Comp
; 16-Bit 2's Complement Operation
;
; inputs: r1, r0 = X
;
; output: r1, r0 = 2's(X)
;
;
; alters: acc, C, register bank 3
;====================================================================
C2Comp:
orl psw, #0x18
mov a, r0	; 1's complement low byte.
cpl a
add a, #1	; And add 1 to do 2's complement.
mov r0, a	; Save result low byte.
mov a, r1	; 2's complement high byte.
cpl a
addc a, #0	; With carry from low byte.
mov r1, a	; Save result high byte.
ret

;====================================================================
; subroutine Mr0r3
; Sign Bit Multiplication.
;
; inputs: MSIGN1 = sign(X): 1 if negative
;         MSIGN0 = sign(Y)
;
; output: MSIGNALL = sign(X x Y)
;
; alters: acc, C, MSIGNALL
;====================================================================
Mr0r3:
jb MSIGN1, Mr0r3b	; Test if X or Y are negative.
jb MSIGN0, Mr0r3a
ret					; Done if X and Y are positive.

Mr0r3b:
jnb MSIGN0, Mr0r3a	; Test if Y is negative.
ret					; Done if X and Y are negative.

Mr0r3a:
setb MSIGNALL		; Set sign bit if X xor Y are negative.
ret

;====================================================================
; subroutine C16toC32
; 16-Bit 2's Complement Cast to 32-Bit 2's Complement
;
; inputs: r1, r0 = X: 16-bit signed (2's complement)
;
; output: r3, r2, r1, r0 = X: 32-bit signed (2's complement)
;
; alters: acc, C
;====================================================================
C16toC32:
mov r3, #0x00		; Pad high bytes with zeros.
mov r2, #0x00
mov a, r1			; Check sign of input.
jnb acc.7, CC32End
mov r3, #0xFF		; Pad high bytes with ones because negative.
mov r2, #0xFF

CC32End:			; Done because positive.
ret

;====================================================================
; subroutine CADD32
; 32-Bit Signed (2's Complement) Addition
;
; inputs: r3, r2, r1, r0 = X
;         r7, r7, r5, r4 = Y
;
; output: r3, r2, r1, r0 = signed sum S = X + Y
;         C is set if overflow occurs
;
; alters: acc, C, OV, register bank 3
;====================================================================
CADD32:
orl psw, #0x18
mov a, r0	; Add low bytes together.
add a, r4
mov r0, a	; Save result in S low byte.
mov a, r1	; Add 2nd bytes together with carry.
addc a, r5
mov r1, a	; Save result in S 2nd byte.
mov a, r2	; Add 3rd bytes together with carry.
addc a, r6
mov r2, a	; Save result in S 3rd byte.
mov a, r3	; Add high bytes together with carry.
addc a, r7
mov r3, a	; Save result in S high byte.
mov C, OV	; Set carry if overflow occurred.
ret

;====================================================================
; subroutine CSUB32
; 32-Bit Signed (2's Complement) Subtraction
;
; inputs: r7, r6, r5, r4 = X
;         r3, r2, r1, r0 = Y
;
; output: r3, r2, r1, r0 = signed difference D = X - Y
;         C is set if overflow occurs
;
; alters: acc, C, OV, register bank 3
;====================================================================
CSUB32:
orl psw, #0x18
clr C
mov a, r4	; Subtract low bytes.
subb a, r0
mov r0, a	; Save result in D low byte.
mov a, r5	; Subtract 2nd bytes with borrow.
subb a, r1
mov r1, a	; Save result in D 2nd byte.
mov a, r6	; Subtract 3rd bytes with borrow.
subb a, r2
mov r2, a	; Save result in D 3rd byte.
mov a, r7	; Subtract high bytes.
subb a, r3
mov r3, a	; Save result in D high byte.
mov C, OV	; Set carry if overflow occurred.
ret

;====================================================================
; subroutine C16toC24
; 16-Bit 2's Complement Cast to 24-Bit 2's Complement
;
; inputs: r1, r0 = X: 16-bit signed (2's complement)
;
; output: r2, r1, r0 = X: 24-bit signed (2's complement)
;
; alters: acc, C
;====================================================================
C16toC24:
orl psw, #0x18
mov r2, #0x00		; Pad high byte with zeros.
mov a, r1
jnb acc.7, CC24End	; Check sign of input.
mov r2, #0xFF		; Pad high byte with ones because negative.

CC24End:			; Done because positive.
ret

;====================================================================
; subroutine CADD24
; 24-Bit Signed (2's Complement) Addition
;
; inputs: r2, r1, r0 = X
;         r6, r5, r4 = Y
;
; output: r2, r1, r0 = signed sum S = X + Y
;         C is set if overflow occurs
;
; alters: acc, C, OV, register bank 3
;====================================================================
CADD24:
orl psw, #0x18
clr C
mov a, r0	; Add low bytes together.
add a, r4
mov r0, a	; Save result in S low byte.
mov a, r1	; Add middle bytes together with carry.
addc a, r5
mov r1, a	; Save result in S middle byte.
mov a, r2	; Add high bytes together.
addc a, r6
mov r2, a	; Save result in S high byte.
mov C, OV	; Set carry if overflow occurred.
ret

;====================================================================
; subroutine CSUB24
; 24-Bit Signed (2's Complement) Subtraction
;
; inputs: r6, r5, r4 = X
;         r2, r1, r0 = Y
;
; output: r2, r1, r0 = signed difference D = X - Y
;         C is set if overflow occurs
;
; alters: acc, C, OV, register bank 3
;====================================================================
CSUB24:
orl psw, #0x18
clr C
mov a, r4	; Subtract low bytes.
subb a, r0
mov r0, a	; Save result in D low byte.
mov a, r5	; Subtract middle bytes with borrow.
subb a, r1
mov r1, a	; Save result in D middle byte.
mov a, r6	; Subtract high bytes with borrow.
subb a, r2
mov r2, a	; Save result in D high byte.
mov C, OV	; Set carry if overflow occurred.
ret

;====================================================================
; subroutine FRound
; 32-bit Signed (2's Complement) Fixed Point Q24.8 Round
;
; XXX: As implemented now, it is Q16.8; no carry to r3 is performed.
;
; inputs: r3, r2, r1. r0 = X
;
; output: r3, r2, r1 = round(X)
;         C is set if overflow occurs
;
; alters: acc, C, OV, register bank 3
;====================================================================
FRound:
orl psw, #0x18
clr C
mov a, r3				; Check if positive.
jnb acc.7, FRPositive
mov a, r0				; Check if changing integer part is necessary.
jb acc.7, FREND
mov a, r1				; Round to next integer (negative).
subb a, #1
mov r1, a				; Store result low byte.
mov a, r2				; Carry borrow to high byte.
subb a, #0
mov r2, a				; Store result high byte.
sjmp FREND				; Done.

FRPositive:
mov a, r0				; Check if changing integer part is necessary.
jnb acc.7, FREND
mov a, r1				; Round to next integer (positive).
addc a, #1
mov r1, a				; Store result low byte.
mov a, r2				; Carry to high byte.
addc a, #0
mov r2, a				; Store result high byte.

FREND:
clr C
ret

; vim: filetype=tasm: tabstop=4:
