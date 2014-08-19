;====================================================================;
;--------------------------------------------------------------------;
; INTERNAL DATA SETUP: Stack bottom must be set to 0x70 in main      ;
;                      program to prevent conflicts.                 ;
;--------------------------------------------------------------------;
;====================================================================;

; System flags.
XGO def 0x21.0		; Motion start/stop flag.
INPOS def 0x21.1	; Servo lock flag.
DATSAV def 0x21.3
MCDIR def 0x21.4	; Motion direction flag.
MSIGN0 def 0x21.5	; Multiplicand sign flag.
MSIGN1 def 0x21.6	; Multiplier sign flag.
MSIGNALL def 0x21.7	; Product sign flag.

; Timing counter value (16-bit).
CNTL epz 0x23
CNTH epz 0x24

; XXX: Buffer address value (16-bit).
BUFF_ADRL def 0x4A
BUFF_ADRH def 0x4B

; Velocity command value VELX (16-bit).
VELXH epz 0x4C
VELXL epz 0x4D

; Position increment/decrement DXC (16-bit).
DXCH epz 0x4E
DXCL epz 0x4F

; Up/Down counter value XC (16-bit).
XCH epz 0x50
XCL epz 0x51

; Position interpolation increment DFX (16-bit).
DFXH epz 0x53
DFXL epz 0x52

; Position following error PEX (16-bit).
PEXH epz 0x54
PEXL epz 0x55

; Residual distance to end position FX (32-bit).
FX1 epz 0x56
FX2 epz 0x57
FX3 epz 0x58
FX4 epz 0x59

; Proportional gains.
KP epz 0x5A		; During motion.
KPINP epz 0x5B	; Servo locked (in position).

; Distance motion command per sampling period DX (16-bit).
DXH epz 0x5E
DXL epz 0x5F

; Absolute position counter value ABSX (32-bit).
ABSX1 epz 0x60
ABSX2 epz 0x61
ABSX3 epz 0x62
ABSX4 epz 0x63

; Miscellaneous Flags.
RDABSEN epz 0x64	; Enable updating software absolute position counter.
INTCNT epz 0x65

; Temporary variable (16-bit) for position inc/decrement calculation.
DABSH epz 0x66
DABSL epz 0x67

; Digital-Analog conversion and control addresses.
_DA1_LOW equ 0xFE18
_DA1_HIGH equ 0xFE19
_DA_CNVT equ 0xFE1F

; Up/Down counter countrol addresses.
_UDCNT1_LOW equ 0xFF05
_UDCNT1_HIGH equ 0xFF06

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
push b					; Save main arithmetic registers to stack.
push acc
orl psw, #0x18			; Use register bank 3 from now on.
mov tl1, #0xE5			; Setup Timer 1 overflow period (10ms).
mov th1, #0xBE
push dpl				; Save data pointer registers to stack.
push dph

;mov dptr, #0xFF00
;mov a, #0xFF
;movx @dptr, a

mov dph, CNTH			; Increment timing counter.
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
pop dph					; Restore staced registers.
pop dpl
pop acc
pop b
pop psw					; PSW automatically pushed by monitor.
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
movx @dptr, a
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
;         r3, r2 = XCH_{i-1}, XCL_{i-1}
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
; 2. Motion Inc/Decrement Calculation and Absolute Counter Update
;
; inputs: XCH, XCL = XC; ABSX4, ABSX3, ABSX2, ABSX1 = ABSX
;
; output: DXCH, DXCL = DXC_i = XC_i - XC_{i-1}
;         If RDABSEN is not set:
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

;====================================================================
; subroutine PEXUpdate
; 3. Update Position Error
;
; inputs: DXCH, DXCL = DXC; DXH, DXL = DX; PEXH, PEXL = PEX_{i-1}
;
; output: PEXH, PEXL = PEX_i = PEX_{i-1} - DXC +/- DX
;
; alters: register bank 3
;====================================================================
PEXUpdate:
mov r1, PEXH		; Subtract current DXC from last PEX.
mov r0, PEXL
mov r3, DXCH
mov r2, DXCL
lcall CSUB16
mov r3, DXH
mov r2, DXL
jnb MCDIR, PlusDir	; Is commanded direction positive?
lcall CSUB16		; If MCDIR == 1, subtract DX from difference.
sjmp OutPEX			; Finish.

PlusDir:
lcall CADD16		; If MCDIR == 0, add DX to difference.

OutPEX:
mov PEXH, r1		; Store result in PEX.
mov PEXL, r0
ret

;====================================================================
; subroutine INPOSJudge
; 4. Determine Required State of Motion Control
;
; inputs: XGO; INPOS
;
; output: INPOS
;====================================================================
INPOSJudge:
jnb XGO, MCStop		; Is motion control active?
clr INPOS			; Then unlock servo.
ret					; And finish.

MCStop:				; Otherwise:
jnb INPOS, SetINPOS	; Is the servo locked?
ret					; Then finish.

SetINPOS:			; If servo not locked,
setb INPOS			; Lock servo.
ret

;====================================================================
; subroutine PPCAL
; 5. Proportional Control Algorithm
;
; inputs: PEXH, PEXL = PEX; KP; KPINP; INPOS

; output: VELXH, VELXL = VELX = KP * PEX if INPOS = 0
;                             = KPINP * PEX if INPOS = 1
;
; alters: acc, register bank 3
;***********************************************************************
PPCAL:
mov a, KP			; Default to in-motion gain KP.
jnb INPOS, CALPP	; Is servo locked?
mov a, KPINP		; If INPOS = 1, use servo locked gain KPINP.

CALPP:
mov r2, a			; Pad gain to 16-bit precision.
mov r3, #0x00
mov r1, PEXH		; And multiply with PEX.
mov r0, PEXL
lcall MUL16
mov VELXL, r0		; Store result in VELX.
mov VELXH, r1
ret

;====================================================================
; subroutine DAOUTVEL
; 6. Offset Correction for DAC Output
;
; inputs: VELXH, VELXL = VELX
;
; output: digital velocity command value VELDA
;
; alters: acc, b, register bank 3
;====================================================================
DAOUTVEL:
jb MSIGNALL, VELNEG	; Is commanded velocity negative?
mov r0, #0xFF		; If positive, add 0x7FFF (offset of 0)
mov r1, #0x7F
mov r2, VELXL
mov r3, VELXH
lcall CADD16
sjmp WROutput		; Done.

VELNEG:
mov r0, #0xFF		; If negative, subtract 0x7FFF.
mov r1, #0x7F
mov r2, VELXL
mov r3, VELXH
lcall CSUB16

WROutput:			; Write offset value to DAC.
mov a, r0
mov b, r1
lcall DAOut
ret

;====================================================================
; subroutine INTPAndFXCheck
; 7. Motion Command Interpolation & Final Position Distance Update
;
; inputs: DFXH, DFXL = DFX; FX4, FX3, FX2, FX1 = FX_i; XGO
;
; output: FX4, FX3, FX2, FX1 = FX_{i+1}
;         DXH, DXL = 0 if XGO = 0; else:
;                  = DFX if FX_{i+1} >= 0
;                  = FX_i otherwise
;         XGO = 0 if FX_{i+1} < 0
;             = 1 otherwise
;
; alters: register bank 3
;====================================================================
INTPAndFXCheck:
jnb XGO, StopINTP	; If motion control is inactive, skip.
mov r1, DFXH		; Otherwise, convert DFX to 32-bit precision.
mov r0, DFXL
lcall C16toC32
mov r7, FX4			; Subtract DFX from FX.
mov r6, FX3
mov r5, FX2
mov r4, FX1
lcall CSUB32
mov FX4, r3			; Store result in FX.
mov FX3, r2
mov FX2, r1
mov FX1, r0
sjmp FXCheckXGO

FXCheckXGO:
mov a, FX3			; Is predicted FX negative?
jb acc.7, CLRXGO	; If yes, we will pass destination.
mov DXH, DFXH		; If not, use standard DX DFX.
mov DXL, DFXL
ret					; Finish.

CLRXGO:
mov DXH, r5			; If will pass, DX is low bytes of FX... XXX?
mov DXL, r4
clr XGO				; Will be done next interrupt.
ret					; Finish.

StopINTP:
mov DXH, #0x00		; Motion control inactive => no motion command.
mov DXL, #0x00
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
push b			; Save extra registers we will use to stack.
push dpl
mov a, r0		; Multiply XL x YL.
mov b, r2
mul ab
push acc		; Stack result low byte.
push b			; Stack result high byte.
mov a, r0		; Multiply XL x YH.
mov b, r3
mul ab
pop 0x18		; Pop to R0.
add a, r0		; Add result low byte to first result high byte.
mov r0, a
clr a
addc a, b		; Add carry to result high byte.
mov dpl, a		; Store carried result high byte in DPL.
mov a, r2		; Multiple XH x YL.
mov b, r1
mul ab
add a, r0		; Add result low byte to R0.
mov r0, a
mov a, dpl
addc a, b		; Add result high byte to DPL.
mov dpl, a
clr a
addc a, #0		; Carry and save to R4.
mov r4, acc
mov a, r3		; Multiply XH x Yh.
mov b, r1
mul ab
add a, dpl		; Add result low byte to DPL.
mov r2, a		; Save product 3rd byte.
mov acc, r4 	; Retrieve carry and add to result high byte.
addc a, b
mov r3, a		; Save product high byte.
mov r1, 0x18	; Save product second byte from R0.
pop 0x18			; Retrieve product low byte.
pop dpl			; Restore registers.
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

; vim: filetype=tasm: tabstop=4:
