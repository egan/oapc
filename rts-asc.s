

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
lcall ADD16
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
lcall ADD16
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
lcall ADD16
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
; subroutine ADD16
; 16-Bit Signed (2's Complement) Addition
;
; input: r1, r0 = X
; r3, r2 = Y
;
; output: r1, r0 = signed sum S = X + Y
; Carry C is set if the result (S) is out of range
;
; alters: acc, C, OV
;====================================================================
 
;org 8670
ADD16:
; orl PSW, #0x18 ; Register Bank 3
mov a, r0 ; load X low byte into acc
add a, r2 ; add Y low byte
mov r0, a ; put result in Z low byte
mov a, r1 ; load X high byte into acc
addc a, r3 ; add Y high byte with carry
mov r1, a ; save result in Z high byte
;mov C, OV
clr C
ret
 
;====================================================================
; subroutine CSUB16
; 16-Bit Signed (2's Complement) Subtraction
;
; input: r1, r0 = X
; r3, r2 = Y
;
; output: r1, r0 = signed difference D = X - Y
; Carry C is set if the result (D) is out of range.
;
; alters: acc, C, OV
;====================================================================
;org 8690
CSUB16: orl psw, #0x18 ; Register Bank 3
mov a, r0 ; load X low byte into acc
clr C ; clear carry flag
subb a, r2 ; subract Y low byte
mov r0, a ; put result in Z low byte
mov a, r1 ; load X high into accumulator
subb a, r3 ; subtract Y high with borrow
mov r1, a ; save result in Z high byte
;mov C, OV
clr C
ret
;====================================================================
; subroutine MUL16
; 16-Bit x 16-Bit to 32-Bit Product Signed Multiply
; 2's Complement format
;
; input: r1, r0 = multiplicand X
; r3, r2 = multiplier Y
;
; output: r3, r2, r1, r0 = product P = X x Y
;
; calls: UMUL16, Cr0r1, Cr2r3, Mr0r3
;
; alters: acc, C, Bits B.0 & B.1
;====================================================================
;org 86e0
MUL16:
 
orl PSW, #0x18 ; Register Bank 3
clr MSIGNALL ;
lcall Cr0r1 ; 2's comp -> Mag/Sign
lcall Cr2r3 ; 2's comp -> Mag/Sign
lcall UMUL16
lcall Mr0r3 ; Mag/Sign -> 2's Comp
;lcall FRound ;
ret
 
;====================================================================
; subroutine UMUL16
; 16-Bit x 16-Bit to 32-Bit Product Unsigned Multiply
;
; input: r1, r0 = multiplicand X
; r3, r2 = multiplier Y
;
; output: r3, r2, r1, r0 = product P = X x Y
;
; alters: acc, C
;====================================================================
;org 86d0
UMUL16: push B
push dpl
mov a, r0
mov b, r2
mul ab ; multiply XL x YL
push acc ; stack result low byte
push b ; stack result high byte
mov a, r0
mov b, r3
mul ab ; multiply XL x YH
pop 18H
add a, r0
mov r0, a
clr a
addc a, b
mov dpl, a
mov a, r2
mov b, r1
mul ab ; multiply XH x YL
add a, r0
mov r0, a
mov a, dpl
addc a, b
mov dpl, a
clr a
addc a, #0
mov r4, acc ; save intermediate carry
mov a, r3
mov b, r1
mul ab ; multiply XH x YH
add a, dpl
mov r2, a
mov acc, r4 ; retrieve carry
addc a, b
mov r3, a
mov r1, 18H
pop 18H ; retrieve result low byte
pop dpl
pop B
ret
 
;===================================================================
; subroutine Cr0r1
; 16-Bit 2's Complement -> magnitude / Sign Bit Conversion
;
; input: r1, r0 = signed word
;
; output: r1, r0 = magnitude
; Bit 21H.4 = sign ( is set if negative number)
;
; alters: acc, C
;===================================================================
;org 8710
Cr0r1: mov a, r1 ; high byte into accumulator
jb acc.7, c0a ; negative if bit 7 is 1
clr MSIGN1 ; clear sign bit if 'positive'
ret ; done
 
c0a: setb MSIGN1
mov a, r0 ; number is negative
cpl a ; complement
add a, #1 ; and add +1
mov r0, a
mov a, r1 ; get next byte
cpl a ; complement
addc a, #0
mov r1, a
ret
 
;====================================================================
; subroutine Cr2r3
; 16-Bit 2's Complement -> magnitude / Sign Bit Conversion
;
; input: r3, r2 = signed word
;
; output: r3, r2 = magnitude
; Bit 21H.3 = sign (set if negative number)
;
; alters: acc, C
;====================================================================
;org 8736
Cr2r3: mov a, r3 ; read high into accumulator
jb acc.7, c1a ; negative if bit 7 is 1
clr MSIGN0 ; clear sign bit if 'positive'
ret ; done
 
c1a: setb MSIGN0 ; set sign flag
mov a, r2 ; number is negative
cpl a ; complement
add a, #1 ; and add +1
mov r2, a
mov a, r3 ; get next byte
cpl a ; complement
addc a, #0
mov r3, a
ret
;====================================================================
; subroutine C2Com
; 16-magnitude / Sign Bit-> 2's Complement Conversion
;
; input: r1, r0 = magnitude / Sign Bit
;
; output: r1, r0 = 2's Complement Conversion
;
;
; alters: acc, C
;====================================================================
 
C2Comp:
mov a, r0 ; number is negative
cpl a ; complement
add a, #1 ; and add +1
mov r0, a
mov a, r1 ; get next byte
cpl a ; complement
addc a, #0h0
mov r1, a
ret
 
;====================================================================
; subroutine Mr0r3
; 32-Bit magnitude / Sign Bit -> 2's Complement Conversion
;
; input: r3, r2, r1, r0 = magnitude
; Bits 21H & 22H = sign bits of operands X and Y
; (set if negative)
;
; output: r3, r2, r1, r0 = signed word
;
; alters: acc, C
;====================================================================
;org 8760
Mr0r3: jb MSIGN1 , Mr0r3b ; test X sign
jb MSIGN0, Mr0r3a ; test Y sign
ret
 
Mr0r3b: jnb MSIGN0, Mr0r3a
ret
 
Mr0r3a:
setb MSIGNALL ;
;mov a, r0 ; negate number
; cpl a ; complement
;add a, #1 ; and add +1
;mov r0, a
;mov a, r1 ; get next byte
;cpl a ; complement
; addc a, #0
; mov r1, a
;mov a, r2 ; get next byte
;cpl a ; complement
;addc a, #0
;mov r2, a
;mov a, r3 ; get next byte
;cpl a ; complement
;addc a, #0
; mov r3, a
ret ; done
 
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
 
