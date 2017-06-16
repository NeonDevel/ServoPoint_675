;;======================================================================;;
;;			ServoPoint					;;
;;======================================================================;;
;;									;;
;; Program:         ServoPoint -- DCC servo controlled point		;;
;; Code:            Paco Cañada						;;
;; Platform:        Microchip PIC12F629, PIC12F675 , 4 Mhz				;;
;; Modifications for PIC12F675 by Arkadiusz Hahn 

;; Date:            26.10.2007						;;
;; First release:   26.10.2007						;;
;; Last release by Paco  26.10.2007
;; LastDate:        16.06.2017						;; 
;;									
;;======================================================================;;
;
;
; Minimal external components, uses internal oscilator at 4 MHz, 
;
; This program is distributed as is but WITHOUT ANY WARRANTY
; I hope you enjoy!!
;
; Revisions:
; 26.10.2007	Start of writting code
; 04.11.2007	change to only button programming 
; 11.11.2007	LED flashing for programming

; Revisions ServoPoint_675
; 15.06.2017  initial version - works on both processors PIC12F675 and PIC12F675
; 16.06.2017  ver 2.0.1 - new hardware version, LED connected to RELE1A, UP/Down buttons on GP4/AN3

; ----- Definitions

#define		__VERNUM	D'1'
#define		__VERDAY	 0x16
#define		__VERMONTH 0x06
#define		__VERYEAR	 0x17





;This project can be compiled either for PIC12F629 or PIC12F675 processors
;   but only PIC12F675 provides full functionality 

  LIST	   p=12F675	; default target processor
; processor 12F675 ; alternative directive for LIST p=
; LIST	   p=12F629	; alternative target CPU
;NOTE:  Processor type is superseded by command line switch: C:\MPASM MYFILE.ASM /PIC12F629    
  

	errorlevel -305,-302

   ; WARNING:  Make sure that internal osc. is calibrated
	 ;           Value has to be read before reprogramming the device.  
  
#ifdef __12F675
  #include p12F675.inc
#endif

#ifdef __12F629
  #include p12F629.inc
#endif

	__CONFIG  _BODEN_ON & _CP_OFF & _WDT_OFF & _MCLRE_OFF & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT 

; --- Constant values

FXTAL		equ	D'4000000'		; internal oscilator

; when needed OSCCAL can be set during programming
;#define OSCCAL_VALUE 0x34

#ifdef __12F629
GP_TRIS         equ     0x0C			; GP2,GP3: inputs
WPU_INI		equ	0x33			; Weak pull-up enable. default, no pull-ups
#endif

#ifdef __12F675
GP_TRIS     equ 0x1C			; GP2,GP3: inputs, GP4: analog input
ANSEL_INI   equ 0x28      ; GP4/AN3 pin - as analog input  ADCS2:0= 010 -> 32/Tosc = 8us conversion time
ADCON0_INI  equ 0x0C      ; AN3 connected to Sample&Hold , VDD - as voltage reference, result Left justified
WPU_INI     equ 0x23			; Weak pull-up enable on outputs GP0,GP1,GP5. default, no pull-ups
#endif

GP_INI          equ     0x00			; all zero
OPTION_INI	equ	0x88			; Option register: no pull-up, falling GP2, no prescaler, wdt 1:1
INTC_INI	equ	0xD0			; GIE, INTE enable, PEIE enable
PIE1_INI	equ	0x01			; interrupt TMR1


#define		RELE1A	GPIO,0			; Rele output straight - RED Led / Programming mode
#define		RELE1B	GPIO,1			; Rele output diverge -  GREEN Led
#define		DCCIN	  GPIO,2			; DCC input pin
#define		SWITCH	GPIO,3			; Move/Programm switch (GPIO,3 only input)
#ifdef __12F629
#define		LED	    GPIO,4			;Prog LED 
#endif

#ifdef __12F675
#define		LED	    GPIO,0			; Prog LED on RELE1A output
#define   KBD     GPIO,4      ; AN3
#endif
#define		SERVO	  GPIO,5			; Servo output


REACH_PULS	equ	0x04			; aditional pulses when reached position


; --- EEPROM Section

#define		EE_INI		0x00

EE_ADDR1H	equ	EE_INI+0x00
EE_ADDR1L	equ	EE_INI+0x01
EE_RANGE1	equ	EE_INI+0x02
EE_SPEED1	equ	EE_INI+0x03

EE_OUT		equ	EE_INI+0x7F		; saved outputs


; ----- Variables

; --- Internal RAM Section

#define		RAMINI0		0x020		; 64 bytes of RAM

INT_W		equ	RAMINI0+0x00		; interrupt context registers
INT_STAT	equ	RAMINI0+0x01

SHIFT0		equ	RAMINI0+0x02
DATA1		equ	RAMINI0+0x03
DATA2		equ	RAMINI0+0x04
DATA3		equ	RAMINI0+0x05
DATA4		equ	RAMINI0+0x06

PREAMBLE	equ	RAMINI0+0x08
DCCSTATE	equ	RAMINI0+0x09
DCCBYTE		equ	RAMINI0+0x0A

EEDATA0		equ	RAMINI0+0x0B		; EEPROM shadow variables
EEADR0		equ	RAMINI0+0x0C


SRVADRH1	equ	RAMINI0+0x10		; servo address
SRVADRL1	equ	RAMINI0+0x11		; 
RANGE1		equ	RAMINI0+0x12		; servo range
SPEED1		equ	RAMINI0+0x13		; servo speed

SRV1CNT		equ	RAMINI0+0x20		; speed
PULSE1		equ	RAMINI0+0x21		; pulse duration
MOVING		equ	RAMINI0+0x22		; moving flags
POSITION	equ	RAMINI0+0x23		; current position flags
STATE		equ	RAMINI0+0x24		; current state
REACHED		equ	RAMINI0+0x25		; position reached
SPACE_H		equ	RAMINI0+0x26		; spacing duration
SPACE_L		equ	RAMINI0+0x27
RELEPULSE1	equ	RAMINI0+0x28		; biestable rele pulse

DEBOUNCE1	equ	RAMINI0+0x2A		; key debouncing
DEBOUNCEP	equ	RAMINI0+0x2B		; prog key debounce
TIMER		equ	RAMINI0+0x2C		; for prescaler prog key debouncing
FLSH_PRE	equ	RAMINI0+0x2D		; flash prescaler
FLSH1		equ	RAMINI0+0x2E		; flash sequence
FLSH2		equ	RAMINI0+0x2F		; 

FLAGS		equ	RAMINI0+0x30

COUNT		equ	RAMINI0+0x32
TEMP		equ	RAMINI0+0x33

EEPTR		equ	RAMINI0+0x3F		; Page register


; --- Flags

#define		NEW_PACKET	FLAGS,0		; New 3 byte packet received
#define		DCC4BYTE	FLAGS,3		; DCC command 4 bytes
#define		DO_PULSE	FLAGS,5		; do pulse, TMR1 end
#define		RESET_FLG	FLAGS,7		; reset packet


; --------------- Program Section --------------------------------------


		org	0x000

PowerUp:
		clrf	STATUS			; Bank 0 default
		clrf	INTCON			; Disable all interrupts
		clrf	PCLATH			; tables on page 0
		goto	INIT

; ----------------------------------------------------------------------

		org	0x004

Interrupt:
		movwf	INT_W			; save context registers		;1
		swapf	STATUS,w							;2
		movwf	INT_STAT							;3
		clrf	STATUS			; interrupt uses bank 0			;4

		btfss	PIR1,TMR1IF		; end of servo pulse?			;+1
		goto	Int_DCC								;+2,3
		bcf	SERVO			; yes, clear pulse			;+3
		bcf	PIR1,TMR1IF							;+4
		bsf	DO_PULSE							;+5

Int_DCC:
		btfss	INTCON,INTF		; RB0 interrupt?			;+4,	+6
		goto	EndInt								;+5,6,	+7,8

		btfss	DCCIN								;5
		goto	Int_Low_Half							;6,7

Int_High_Half:
		movf	DCCSTATE,w							; 8
		addwf	PCL,f								; 9

		goto	Preamble							; 10,11
		goto	WaitLow
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadLastBit
		goto	EndByte1
		goto	EndByte2
		goto	EndByte3
		goto	EndByte4

Int_Low_Half:
		movlw	d'256' - d'80'		; 77us: between 64us (one) and 90us (zero);8
		movwf	TMR0								;9
		bcf	INTCON,T0IF		; clear overflow flag for counting	;10
		bcf	INTCON,INTF							;11
		bsf	STATUS,RP0							;12
		bsf	OPTION_REG,INTEDG	; next interrupt on rising edge GP2	;13
		swapf	INT_STAT,w		; restore context registers		;14
		movwf	STATUS								;15
		swapf	INT_W,f								;16
		swapf	INT_W,w								;17
		retfie									;18,19

EndHighHalf:
		bcf	INTCON,INTF							;21
		bsf	STATUS,RP0							;22
		bcf	OPTION_REG,INTEDG	; next interrupt on falling edge GP2	;23
EndInt:
		swapf	INT_STAT,w		; restore context registers		;24
		movwf	STATUS								;25
		swapf	INT_W,f								;26
		swapf	INT_W,w								;27
		retfie									;28,29


Preamble:
		btfss	NEW_PACKET		; wait until last decoded		;12
		incf	PREAMBLE,f		;					;13
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;14
		clrf	PREAMBLE		;					;15
		movlw	0xF6			; 10 preamble bits?			;16
		addwf	PREAMBLE,w		;					;17
		btfsc	STATUS,C		;					;18
		incf	DCCSTATE,f		; yes, next state			;19
		goto	EndHighHalf		;					;20,21
		

WaitLow:
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;12
		incf	DCCSTATE,f		; then state				;13
		clrf	DCCBYTE			;					;14
		clrf	PREAMBLE		;					;15
		clrf	DATA4			;					;16
		goto	EndHighHalf		;					;17,18


ReadBit:
		bsf	STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf	STATUS,C							;14
		rlf	SHIFT0,f		; receiver shift register		;15
		incf	DCCSTATE,f		;					;16
		goto	EndHighHalf		;					;17,18
			
ReadLastBit:
		bsf	STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf	STATUS,C							;14
		rlf	SHIFT0,f		; receiver shift register		;15
		incf	DCCBYTE,w							;16
		addwf	DCCSTATE,f							;17
		goto	EndHighHalf		;					;18,19

EndByte1:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA1			;					;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20

EndByte2:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA2			;					;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20


EndByte3:
		btfss	INTCON,T0IF		; End bit=1, end of packet		;12
		goto	EndByte3x		;					;13,14
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA3			;					;17
		incf	DCCBYTE,f		;					;18
		bsf	DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21
EndByte3x:
		clrf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA3			;					;17
		bsf	NEW_PACKET		;					;18
		bcf	DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21

EndByte4:
		clrf	DCCSTATE		;					;12
		btfsc	INTCON,T0IF		; End bit=1, end of packet		;13
		goto	EndInt			; End bit=0, invalid packet		;14,15
		movf	SHIFT0,w		;					;15
		movwf	DATA4			;					;16
		bsf	NEW_PACKET		;					;17
		goto	EndHighHalf		;					;18,19


; ----------------------------------------------------------------------

; ----- Initialization

INIT:
		clrf	GPIO
		movlw	0x07
		movwf	CMCON			; set GP2:0 to digital I/O
#ifdef __12F675     
    movlw ADCON0_INI
    movwf ADCON0
#endif
		bsf	STATUS,RP0		; bank 1
#ifdef __12F675 
    movlw ADCON0_INI
    movwf ANSEL     ; PIC12F675  Analog Select Register
#endif   
		movlw	GP_TRIS
		movwf	TRISIO
#ifndef OSCCAL_VALUE    
		call	0x3FF			; get factory calibrated OSCCAL from last address of Flash
#else
    movlw OSCCAL_VALUE   ; set OSCCAL manualy 
#endif
		movwf	OSCCAL
		movlw	WPU_INI			; pull-ups
		movwf	WPU
		clrf	IOC			; interrupt on change
		clrf	VRCON			; voltage reference off
		movlw	OPTION_INI		; Option register: no pull-up, falling GP2, no prescaler, wdt 1:1
		movwf	OPTION_REG
		movlw	PIE1_INI
		movwf	PIE1
		bcf	STATUS,RP0		; bank 0
		clrf	PIR1
		movlw	0x01			; Timer 1 on, 1:1
		movwf	T1CON


		movlw	0x20			; clear RAM
		movwf	FSR
ClearRAM:
		clrf	INDF
		incf	FSR,f
		movlw	0x60
		xorwf	FSR,w
		btfss	STATUS,Z
		goto	ClearRAM

		movlw	INTC_INI
		movwf	INTCON			; enable GP2 external interrupt

		movlw	d'150'			; init servos default
		movwf	PULSE1
;		clrf	MOVING
;		clrf	POSITION
;		clrf	STATE

		call	LoadCV			; Load CV in SFR
		call	LoadOutputs		; set servo to last position


; ----------------------------------------------------------------------


MainLoop:
		btfsc	NEW_PACKET		; new packet?
		call	Decode			; yes, decode

		btfsc	DO_PULSE		; end of servo pulse?
		call	DoServo			; yes, next pulse

		goto	MainLoop


DoServo:
		bcf	DO_PULSE
		btfsc	INTCON,INTE		; disabled interrupts?
		goto	DoServoJump
		clrf	DCCSTATE		; yes, clear for decoding
		bsf	INTCON,INTE		; re-enable interrupts
DoServoJump:
		movf	STATE,w
		andlw	0x03
		addwf	PCL,f

		goto	PulseServo1
		goto	PulseServo2
		goto	Spacing
		clrf	STATE			; prevents erroneus contents


	if ($ > d'255') 
		ERROR "  Tables exceded page 0.   If it works, why do you change it?   "
	endif

; ----------------------------------------------------------------------

PulseServo1:
		clrf	SPACE_H			; init spacing value (20ms)
		clrf	SPACE_L

		movf	PULSE1,w		; 200: 2ms, 100: 1ms
		call	PulseServo

		btfsc	MOVING,0
		bcf	INTCON,INTE		; disable DCC interrupts for time accuracy

		btfsc	MOVING,0
		bsf	SERVO

		bsf	T1CON,TMR1ON		; run timer 1
		incf	STATE,f

		movf	RELEPULSE1,w		; relay pulse
		btfss	STATUS,Z
		decfsz	RELEPULSE1,f
		goto	EndSpacing
		bcf	RELE1A
		bcf	RELE1B
		goto	EndSpacing


PulseServo2:
		movlw	d'150'			; 200: 2ms, 100: 1ms
		call	PulseServo		; dummy pulse
		bsf	T1CON,TMR1ON		; run timer 1
		incf	STATE,f

;		movf	RELEPULSE2,w		; relay pulse
;		btfss	STATUS,Z
;		decfsz	RELEPULSE2,w
;		goto	EndServo1
;		bcf	RELE2A
;		bcf	RELE2B
		goto	EndServo1


Spacing:
		bcf	T1CON,TMR1ON
		movf	SPACE_H,w		; pulses length
		movwf	TMR1H
		movf	SPACE_L,w
		movwf	TMR1L
		movlw	0xE0			; 20ms time ( 0x10000 - 0x4E20 = 0xB1E0 )
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
		movlw	0xB1
		addwf	TMR1H,f		
;		bcf	PIR1,TMR1IF
		bsf	T1CON,TMR1ON		; run timer 1
		clrf	STATE
		goto	EndServo2


PulseServo:
		bcf	T1CON,TMR1ON		; stop timer 1
		clrf	TMR1H			; do PULSE x10
		movwf	TEMP
		movwf	TMR1L
		rlf	TMR1L,f
		rlf	TMR1H,f
		rlf	TMR1L,f			; x4
		rlf	TMR1H,f
		rlf	TMR1L,f			; x8
		rlf	TMR1H,f
		movlw	0xF8
		andwf	TMR1L,f
		movf	TEMP,w			; x8 + x1 = x9
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
		addwf	TMR1L,f			; x9 + x1 = x10
		btfsc	STATUS,C
		incf	TMR1H,f

		movf	TMR1L,w			; correct spacing
		addwf	SPACE_L,f
		btfsc	STATUS,C
		incf	SPACE_H,f
		movf	TMR1H,w
		addwf	SPACE_H,f

		comf	TMR1L,f			; negative for incrementing
		comf	TMR1H,f
		movlw	0x01
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
;		bcf	PIR1,TMR1IF
		return


; ----------------------------------------------------------------------


EndSpacing:					; every 20ms
		return


EndServo1:
		btfss	MOVING,0		; moving servo?
		return				; no

		btfss	POSITION,0		; yes	*************
		goto	EndServo1Nxt
		
		btfsc	REACHED,0
		goto	EndServo1WW1

		decfsz	SRV1CNT,f		; speed
		goto	EndServo1W1
		movf	SPEED1,w
		movwf	SRV1CNT
		decf	PULSE1,f
EndServo1W1:
		comf	RANGE1,w		; range (negative)
		addlw	0x01
		addlw	d'150'
		xorwf	PULSE1,w
		btfss	STATUS,Z
		return
		movlw	REACH_PULS
		movwf	SRV1CNT
		bsf	REACHED,0
		return
EndServo1WW1:
		decfsz	SRV1CNT,f		; additional pulses
		return
		bcf	POSITION,0		; **********************
EndServoSave:
		bcf	REACHED,0
		bcf	MOVING,0

		movlw	d'5'			; 5 * 20ms relay pulse
		movwf	RELEPULSE1
		btfss	POSITION,0
		bsf	RELE1A
		btfsc	POSITION,0
		bsf	RELE1B

		movf	POSITION,w		; save new output state
		xorlw	0x01	
		movwf	EEDATA0
		movlw	EE_OUT
		goto	SetParm

EndServo1Nxt:
		btfsc	REACHED,0
		goto	EndServo1WW2

		decfsz	SRV1CNT,f		; speed
		goto	EndServo1W2
		movf	SPEED1,w
		movwf	SRV1CNT
		incf	PULSE1,f
EndServo1W2:
		movlw	d'150'			; range
		addwf	RANGE1,w
		xorwf	PULSE1,w
		btfss	STATUS,Z
		return
		movlw	REACH_PULS
		movwf	SRV1CNT
		bsf	REACHED,0
		return
EndServo1WW2:
		decfsz	SRV1CNT,f
		return
		bsf	POSITION,0		; *****************
		goto	EndServoSave


EndServo2:					; every 20ms
		btfsc	SWITCH			; check program switch 
		goto	ReadInputSwitch
		decf	DEBOUNCE1,f		; *** for 2,5s
		decfsz	DEBOUNCE1,f		; pressed, wait debounce time (2,5s)
		return
		goto	MainProg		; enter programming mode
ReadInputSwitch:
		movf	DEBOUNCE1,w		; short pressed?
		btfsc	STATUS,Z
		return				; no
Key_Change:
						; servo move
		bsf	MOVING,0
		movf	SPEED1,w
		movwf	SRV1CNT
		clrf	DEBOUNCE1
		return



; ----------------------------------------------------------------------


MainProg:
		bsf	LED			; LED on
		btfss	SWITCH			; wait to release
		goto	MainProg
		clrf	EEPTR
		bsf	FLSH_PRE,1
		call	SetFlash
ProgLoop:
		btfss	DO_PULSE		; every 65ms
		goto	ProgKey
		bcf	DO_PULSE
		decfsz	FLSH_PRE,f		; prescaler flash
		goto	ProgKey
		bsf	FLSH_PRE,1
		bcf	STATUS,C
		btfsc	FLSH1,7
		bsf	STATUS,C
		rlf	FLSH2,f
		rlf	FLSH1,f
		btfss	STATUS,C
		bcf	LED
		btfsc	STATUS,C
		bsf	LED

ProgKey:
		decfsz	TIMER,f			; debounce time
		goto	ProgNoKey
		bsf	TIMER,2
		bcf	STATUS,C		; check key
		btfsc	SWITCH
		bsf	STATUS,C
		rlf	DEBOUNCEP,w
		movwf	DEBOUNCEP
		xorlw	0xC0
		btfss	STATUS,Z
		goto	ProgNoKey
		btfss	EEPTR,1
		incf	EEPTR,f
		incf	EEPTR,f
		call	SetFlash
		btfss	EEPTR,2
		goto	ProgLoop
EndProg:
		bcf	LED
		bsf	INTCON,INTE		; enable interrupts
		return

ProgNoKey:
		btfss	NEW_PACKET		; new packet?
		goto	ProgLoop

DecodeProg:
		bcf	NEW_PACKET		; prepare for next packet
		bcf	INTCON,INTE		; disable interrupts for more speed

		movf	DATA1,w			; exclusive or check
		xorwf	DATA2,w
		xorwf	DATA3,w
		xorwf	DATA4,w

		btfss	STATUS,Z		; valid packet?
		goto	ExitDecodeProg		; no, return

		movf	DATA1,w			;'10AAAAAA'  '1AAADxxx' AAA:111 D:1 activate
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfsc	DATA2,7
		btfss	STATUS,Z
		goto	ExitDecodeProg
		btfss	DATA2,3			; activate?
		goto	ExitDecodeProg

		btfss	EEPTR,1
		goto	ProgSetAddr		; address
ProgRangeSpeed:
		decf	DATA1,f			; '10AAAAAA'
		rlf	DATA1,w
		movwf	TEMP
		rlf	TEMP,w			; 'AAAAAAxx'
		andlw	0xFC
		movwf	TEMP	

		rrf	DATA2,w			; '1AAA1CCD'
		andlw	0x03
		iorwf	TEMP,f
		incf	TEMP,w
		movwf	EEDATA0

		movf	EEPTR,w
		call	SetParm
		call	LoadCV
		bsf	MOVING,0		; move to test
		movf	SPEED1,w
		movwf	SRV1CNT
		goto	EndProg


ProgSetAddr:
		movf	DATA1,w
		movwf	EEDATA0
		movlw	EE_ADDR1H
		call	SetParm			; save address
		movf	DATA2,w
		movwf	EEDATA0
		movlw	EE_ADDR1L
		call	SetParm			; save address
		call	LoadCV
		bsf	MOVING,0		; move to test
		movf	SPEED1,w
		movwf	SRV1CNT
		goto	EndProg

ExitDecodeProg:
		bsf	INTCON,INTE		; enable interrupts
		goto	ProgLoop



SetFlash:
		btfss	EEPTR,1
		goto	FlashLED
		movlw	b'11111111'		
		btfsc	EEPTR,0
		movlw	b'10101010'
		movwf	FLSH1
		movwf	FLSH2
		return
FlashLED:
		movlw	b'11000000'
		movwf	FLSH1
		movwf	FLSH2
		return


; ----------------------------------------------------------------------

Decode:
		bcf	NEW_PACKET		; prepare for next packet
		bcf	INTCON,INTE		; disable interrupts for more speed

		movf	DATA1,w			; exclusive or check
		xorwf	DATA2,w
		xorwf	DATA3,w
		xorwf	DATA4,w

		btfss	STATUS,Z		; valid packet?
		goto	ExitDecode		; no, return

; 'AAAAAAAA''DDDDDDDD''EEEEEEEE'		; 3 byte packet
;   DATA1     DATA2     DATA3

		movf	DATA1,w			; address = '00000000' ?
		btfsc	STATUS,Z
		goto	Broadcast
;		movf	DATA1,w
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfss	STATUS,Z
		goto	ExitDecode
		goto	Accessory

Accessory:
		bcf	RESET_FLG
;		btfsc	KEY_PROG		; pressed switch?
;		goto	AccessProg		; yes, program new address

AccSrv1:
		movf	SRVADRH1,w		; '10AAAAAA' of servo 1
		xorwf	DATA1,w
		btfss	STATUS,Z
		goto	AccSrv2
		movf	SRVADRL1,w		; '1AAACDDD'
		xorwf	DATA2,w
		btfss	STATUS,Z
		goto	AccSrv1B
		btfss	POSITION,0		; move servo
		bsf	MOVING,0
		goto	AccSrv1C
AccSrv1B:
		xorlw	0x01			; other position?
		btfss	STATUS,Z
		goto	AccSrv2
		btfsc	POSITION,0		; move servo
		bsf	MOVING,0
AccSrv1C:
		movf	SPEED1,w		; set speed
		movwf	SRV1CNT

AccSrv2:
		goto	ExitFunction


ExitFunction:
		bcf	RESET_FLG
		bsf	INTCON,INTE		; enable DCC interrupts
		return


; -----------------------------------------------------------------------

Broadcast:
		movf	DATA2,w			; reset packet?
		btfss	STATUS,Z
		goto	ExitDecode
;		bcf	PROG_2X
		bsf	RESET_FLG

Clear:						; reset decoder

;		bcf	SERVO			; servo disable

		bsf	INTCON,INTE		; enable interrupts
		return


ExitDecode:
		bcf	RESET_FLG
		bsf	INTCON,INTE		; enable interrupts
		return


;---------------------------------------------------------------------------


LoadCV:
		movlw	SRVADRH1		; first CV to read
		movwf	FSR
		movlw	0x00
		movwf	EEADR0
LoadCVNxt:
		movf	EEADR0,w
		call	EE_Read
		movwf	INDF
		movlw	SPEED1			; last CV to read
		xorwf	FSR,w
		btfsc	STATUS,Z
		goto	LoadCVEnd
		incf	FSR,f
		incf	EEADR0,f
		goto	LoadCVNxt
LoadCVEnd:

		return


;---------------------------------------------------------------------------

LoadOutputs:
		movlw	EE_OUT			; read saved outputs
		call	EE_Read
		movwf	POSITION			

		movf	RANGE1,w		; calculate pulse for position
		btfsc	POSITION,0		; ***************
		xorlw	0xFF
		btfsc	POSITION,0		; ***************
		addlw	0x01
		addlw	d'150'
		movwf	PULSE1

		movlw	0x10			; pulses for setting position
		movwf	SRV1CNT
		bsf	REACHED,0		; do move to reached position
		bsf	MOVING,0

		return


;----- Internal EEPROM routines ------------------------------------------------


EE_Read:
		bsf	STATUS,RP0		; w=ADR
		movwf	EEADR
		bsf	EECON1,RD
		movf	EEDATA,w
		bcf	STATUS,RP0
		return

SetParm:
		call	EE_Read			; w=ADR, EEDATA0=data. Write only changes
		xorwf	EEDATA0,w
		btfsc	STATUS,Z
		return
EE_Write:		
		movf	EEDATA0,w
		bsf	STATUS,RP0
		movwf	EEDATA
		bsf	EECON1,WREN
		bcf	INTCON,GIE
		movlw	0x55
		movwf	EECON2
		movlw	0xAA
		movwf	EECON2
		bsf	EECON1,WR
		bsf	INTCON,GIE
		bcf	EECON1,WREN
EEWrite0:
		btfsc	EECON1,WR
		goto	EEWrite0
		bcf	STATUS,RP0
		return

;---------------------------------------------------------------------------


; ----- EEPROM default values


		org	0x2100


						;'10AAAAAA'1AAACDDD'
		dw	0x81			; address
		dw	0xF8
		dw	0x32			; range
		dw	0x01			; speed



		org	0x2120

		dt	" Servo  "
		dt	" Point  "
		dt	"A. Hahn "
		dt	(__VERDAY   >> 4)  +0x30
		dt	(__VERDAY   & 0x0F)+0x30,"/"
		dt	(__VERMONTH >> 4)  +0x30
		dt	(__VERMONTH & 0x0F)+0x30,"/"
		dt	(__VERYEAR  >> 4)  +0x30
		dt	(__VERYEAR  & 0x0F)+0x30

		org	0x217F

		dw	0x00			; last state

	end
