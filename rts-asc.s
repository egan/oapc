

;variable defined in internal data ram starting from 4E Unit
;Stack bottom must be set to 70H in main program to prevent
;a confilict between assembly variables and stack space

;50H, 51H current Up/Down counter Value-----------------> Xc(i) pulse
XCHI epz 50H
XCL epz 51H

;52H, 53H interpolation position increment --------------> DFX Pulse
DFXH epz 53H
DFXL epz 52H

;54H, 55H position following error --------> PEX(i) Pulse
PEXH epz 54H
PEXL epz 55H

;56H, 57H ,58H,59H Residual distance ---------------> Fx(i) Pulse
FX4 epz 59H
FX3 epz 58H
FX2 epz 57H
FX1 epz 56H

;5AH Proportioanl gain ---------------> Kp
KP epz 5AH

;5BH Servo locked Proportioanl gain ---------------> Kpinp
KPINP epz 5BH

; 5EH, 5FH Distance motion command per sampling period --------------> DX Pulse
DXH epz 5EH
DXL epz 5FH

;60H,61H,62H,63H current absolute position counter-----------------------ABX Pusle
ABSX4 epz 63H
ABSX3 epz 62H
ABSX2 epz 61H
ABSX1 epz 60H

RDABSEN epz 64H
INTCNT epz 65h

;66H,67H Temporary variables
DABSH epz 66h
DABSL epz 67h

;4CH,4DH Velocity Command Value-------------------------VELX
VELXH epz 4CH
VELXL epz 4DH

; 4EH, 4FH Position increment/decrement -------------- delta Xc(i) Pulse
DXCH epz 4EH
DXCL epz 4FH

XGO def	021H.0 ; motion start/stop flag
INPOS def	021H.1 ; used for servo lock flag
MCDIR def	021H.4 ; direction command from PC
DATSAV def	021H.3

MSIGN0 def	021H.5
MSIGN1 def	021H.6
MSIGNALL def	021H.7
BUFF_ADRL def 04AH
BUFF_ADRH def 04BH
_DA1_LOW equ 0FE18h
_DA1_HIGH equ 0FE19h
_DA_CNVT equ 0FE1Fh

_UDCNT1_LOW equ 0FF05h
_UDCNT1_HIGH equ 0FF06h

;***********************************************************************
; Timer interrupt Initial
;
;***********************************************************************
org 0x9F00
push psw
orl psw, #0x18	; use register bank 3 from now on
push acc
push b
push dpl
push dph

mov dptr, #0xFA1B	; set Timer1 interrupt vector
mov a, #0x02	;ljump timer interrupt subroutine address
movx @dptr, a
inc dptr
mov a, #0x9F	;LJMP #0x
movx @dptr, a
inc dptr
mov a, #0x50
movx @dptr, a
mov tmod, #0x10	;16bit Timer/Counter
mov tl1, #0E5h	;20Mhz,3ms (EC78h),10ms(BEE5h),
mov th1, #0BEh	;20Mhz,1ms F97DH
lcall ClearCounter
mov a, #000h
mov b,#080h
lcall DAOUT
lcall INITVAR;
setb tr1
orl ie,#0x88

pop dph
pop dpl
pop b
pop acc
pop psw
ret


;***********************************************************************
; Timer interrupt subroutine
;
;***********************************************************************
org 0x9F50
push B
push acc
orl psw, #0x18
mov tl1, #0E5h	;20Mhz,3ms (EC78h),10ms(BEE5h),
mov th1, #0BEh	;20Mhz,1ms F97DH
push dpl
push dph

;mov dptr, #0xFF00
;mov a, #0xFF
;movx @dptr, a

lcall ReadUDCounter ;module 1
lcall PosDiffCAL ;module 2
lcall PEXUpdate ;module 3
lcall INPOSJudge ;module 4
lcall	PPCAL ;module 5
lcall	DAOUTVEL ;module 6
lcall INTPAndFXCheck ;module 7,8

;mov dptr, #0xFF00
;mov a, #0x00
;movx @dptr, a

done:
pop dph
pop dpl
pop acc
pop B
pop psw
reti


;***********************************************************************
; Initial variables
;
;
;***********************************************************************
INITVAR:
mov ABSX4,#00h
mov ABSX3,#00h
mov ABSX2,#00h
mov ABSX1,#00h
mov PEXH,#00h
mov PEXL,#00h
mov DXH,#00h
mov DXL,#00h
mov XCHI,#00h ;---> FIRST PROBLEMATIC LINE NO ">"
mov XCL,#00h ;--> crashes MPS
setb INPOS
clr DATSAV
clr XGO
mov INTCNT ,#00h
mov RDABSEN,#00h
mov DABSH ,#00h
mov DABSL ,#00h
ret
;***********************************************************************
; Clear position counter
;Input:
;
;***********************************************************************
ClearCounter:
mov dptr, #_UDCNT1_HIGH ; Counter1
mov a,#00h
movx @dptr,a
ret

;***********************************************************************
; D/A output
;Input:
; High byte (B), Low byte (ACC)
;
;***********************************************************************
DAOut:
mov dptr, #_DA1_LOW
movx @dptr,a
nop
nop
mov a,b
mov dptr, #_DA1_HIGH
movx @dptr,a
nop
nop
mov dptr, #_DA_CNVT ;start conversion
movx @dptr,a
ret

;***********************************************************************
;1. Read Up/Down counter
;
;Output: XCHI --- HIGH byte of couter value
; XCL --- LOW byte
;
;***********************************************************************
ReadUDCounter:
mov r3,XCHI ;count value atlast period
mov r2,XCL
mov dptr,#_UDCNT1_LOW ;enable start read
mov a,#00h
movx @dptr,a
mov dptr,#_UDCNT1_LOW
movx a, @dptr
mov XCL,a ;Read LOW byte
mov dptr,#_UDCNT1_HIGH
movx a, @dptr
mov XCHI,a ;Read HIGH byte
ret

;***********************************************************************
;2. Motion increment/ decrement calculation and update absolute counter
;
;Input:
; XCHI: Last counter value HIGH byte
; XCL: Last counter value LOW byte
; ABSX4: Last absolute value 4TH byte
; ABSX3: Last absolute value 3RD byte
; ABSX2: Last absolute value 2ND byte
; ABSX1: Last absolute value 1ST byte
;Output:
; DXCH: Count difference at adjacent sampling time HIGH byte
; DXCL: Count difference at adjacent sampling time LOW byte
; ABSX4: Current absolute value 4TH byte
; ABSX3: Current absolute value 3RD byte
; ABSX2: Current absolute value 2ND byte
; ABSX1: Current absolute value 1ST byte
;
;***********************************************************************
PosDiffCAL:
mov r1,XCHI
mov r0,XCL
lcall CSUB16
mov DXCH,r1
mov DXCL,r0

mov r3,DABSH
mov r2,DABSL
lcall CADD16
mov DABSH,r1
mov DABSL,r0
mov a, RDABSEN
xrl a,#01h	;Read absolute count value
jnz ABXUpdate
ret
ABXUpdate:
mov r1,DABSH
mov r0,DABSL
lcall C16toC24 ;r0,r1,r2,r3

mov r7,ABSX4
mov r6,ABSX3
mov r5,ABSX2
mov r4,ABSX1

lcall CADD24 ;calculate the absolute position

mov ABSX4,r3
mov ABSX3,r2
mov ABSX2,r1
mov ABSX1,r0

mov DABSH,#00h
mov DABSL,#00h
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
;org 8670
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
;org 8690
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
;org 86e0
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
;org 86d0
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
;org 8710
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
;org 8736
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
; subroutine C2Com
; 16-Bit 2's Complement Operation
;
; inputa: r1, r0 = X
;
; output: r1, r0 = 2's(X)
;
;
; alters: acc, C, register bank 3
;====================================================================
; org XXXX
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
; Sign bit multiplication.
;
; inputs: MSIGN1 = sign(X): 1 if negative
;         MSIGN0 = sign(Y)
;
; output: MSIGNALL = sign(X x Y)
;
; alters: acc, C, MSIGNALL
;====================================================================
;org 8760
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
;16-Bit 2's Complement -> 32-Bit 2's Complement Conversion
;SC16TOSC32
;Input:
;
; r1, r0=X: 16-Bit signed data (2's Complement)
; r1 is X HIGH byte, r0 is X LOW byte
;
;Output:
; r3, r2, r1, r0=X: 32-Bit signed data (2's Complement)
;
;Alters: ACC, C
;====================================================================
C16toC32:
mov r3,#00h
mov r2,#00h
mov a,r1
jnb acc.7, CC32End
mov r3,#0FFh
mov r2,#0FFh
CC32End:
ret

;====================================================================
; subroutine CADD32
; 32-Bit Signed (2's Complement) Addition
;
; input:
; r3, r2, r1, r0=X: 32-Bit signed data (2's Complement),
; r7, r6, r5, r4=Y: 32-Bit signed data (2's Complement)
;
; output:
; r3, r2, r1, r0=S: Signed sum (2's Complement), S=X+Y
; Carry C is set if the result (S) is out of range
;
; alters: acc, C, OV
;====================================================================

;org 8670
CADD32:
orl PSW, #0x18 ; Register Bank 3
clr C ; clear carry flag
mov a, r0 ; load X low byte into acc
addc a, r4 ; add Y low byte
mov r0, a ; put result in Z low byte

mov a, r1 ; load X low byte into acc
addc a, r5 ; add Y low byte
mov r1, a ; put result in Z low byte

mov a, r2 ; load X low byte into acc
addc a, r6 ; add Y low byte
mov r2, a ; put result in Z low byte

mov a, r3 ; load X low byte into acc
addc a, r7 ; add Y low byte
mov r3, a ; put result in Z low byte

mov C, OV

ret
;====================================================================
;====================================================================
; subroutine CSUB32
; 32-Bit Signed (2's Complement) Subtract
;
; input:
; r7, r6, r5, r4=X: 32-Bit signed data (2's Complement),
; r3, r2, r1, r0=Y: 32-Bit signed data (2's Complement)
;
; output:
; r3, r2, r1, r0=S: Signed sum (2's Complement), S=X-Y
; Carry C is set if the result (S) is out of range
;
; alters: acc, C, OV
;====================================================================

;org 8670
CSUB32:

orl PSW, #0x18 ; Register Bank 3
clr C ; clear carry flag
mov a, r4 ; load X low byte into acc
subb a, r0 ; add Y low byte
mov r0, a ; put result in Z low byte

mov a, r5 ; load X low byte into acc
subb a, r1 ; add Y low byte
mov r1, a ; put result in Z low byte

mov a, r6 ; load X low byte into acc
subb a, r2 ; add Y low byte
mov r2, a ; put result in Z low byte

mov a, r7 ; load X low byte into acc
subb a, r3 ; add Y low byte
mov r3, a ; put result in Z low byte

mov C, OV

ret
;====================================================================
;16-Bit 2's Complement -> 24-Bit 2's Complement Conversion
;SC16TOC24
;Input:
;
; r1, r0=X: 16-Bit signed data (2's Complement)
; r1 is X HIGH byte, r0 is X LOW byte
;
;Output:
; r2, r1, r0=X: 24-Bit signed data (2's Complement)
;
;Alters: ACC, C
;====================================================================
C16toC24:
mov r2,#00h
mov a,r1
jnb acc.7, CC24End
mov r2,#0FFh
CC24End:
ret

;====================================================================
; subroutine CADD24
; 24-Bit Signed (2's Complement) Addition
;
; input:
; r2, r1, r0=X: 32-Bit signed data (2's Complement),
; r6, r5, r4=Y: 32-Bit signed data (2's Complement)
;
; output:
; r2, r1, r0=S: Signed sum (2's Complement), S=X+Y
; Carry C is set if the result (S) is out of range
;
; alters: acc, C, OV
;====================================================================

;org 8670
CADD24:
orl PSW, #0x18 ; Register Bank 3
clr C ; clear carry flag
mov a, r0 ; load X low byte into acc
addc a, r4 ; add Y low byte
mov r0, a ; put result in Z low byte

mov a, r1 ; load X low byte into acc
addc a, r5 ; add Y low byte
mov r1, a ; put result in Z low byte

mov a, r2 ; load X low byte into acc
addc a, r6 ; add Y low byte
mov r2, a ; put result in Z low byte

mov C, OV

ret

;====================================================================
; subroutine CSUB24
; 24-Bit Signed (2's Complement) Subtract
;
; input:
; r6, r5, r4=X: 24-Bit signed data (2's Complement),
; r2, r1, r0=Y: 24-Bit signed data (2's Complement)
;
; output:
; r2, r1, r0=S: Signed sum (2's Complement), S=X-Y
; Carry C is set if the result (S) is out of range
;
; alters: acc, C, OV
;====================================================================

CSUB24:

orl PSW, #0x18 ; Register Bank 3
clr C ; clear carry flag
mov a, r4 ; load X low byte into acc
subb a, r0 ; add Y low byte
mov r0, a ; put result in Z low byte

mov a, r5 ; load X low byte into acc
subb a, r1 ; add Y low byte
mov r1, a ; put result in Z low byte

mov a, r6 ; load X low byte into acc
subb a, r2 ; add Y low byte
mov r2, a ; put result in Z low byte

mov C, OV

ret

;====================================================================
; subroutine ROUND
; fixed float Signed (2's Complement) Round
;
; input:
; r3,r2,r1,r0=X: fixed float signed data (2's Complement),
; r3r2r1.r0: (if r0>0.5)
;
; output:
; r3r2r1=S : integer
; Carry C is set if the result (S) is out of range
;
; alters: acc, C, OV
;====================================================================

;org 8670
FRound:
orl PSW, #0x18 ; Register Bank 3
clr C ; clear carry flag
mov a,r3

jnb acc.7, FRPositive

mov a,r0 ; r0 > 0.5 (80h)
jb acc.7, FREND
mov a, r1
subb a, #01h
mov r1,a
mov a,r2
subb a,#00h
mov r2,a
sjmp FREND
FRPositive:
mov a,r0 ; r0 > 0.5 (80h)
jnb acc.7, FREND
mov a, r1
addc a, #01h
mov r1,a
mov a,r2
addc a,#00h
mov r2,a

FREND:
clr c
ret

; vim: filetype=tasm
