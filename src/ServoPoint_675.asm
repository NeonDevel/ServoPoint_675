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
;; LastDate:        11.07.2017						;; 
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
; 20.06.2017  ver 2.0.2 - added new variables, and ADC interrupt handler. Bug since v 2.0.1 - DCC doesn't work
; 25.06.2017 	ver 2.1.1	- Pic12F675 version only, 
; 											- DCC decoding fixed, 
; 											- Speed and Range configuration using Up/Down buttons
; 											- Speed settings limit from 1 fastest to 4 slowest, default 1
; 											- Range from 0 to 50, default 25
; 26.06.2017 ver 2.1.2	- fixed flash prescaller, 10ms PPM period
; 03.07.2017 ver 2.1.4	- (speed range (1..4), positive angle range (2..50) [0.1ms] with step 2   
; 11.07.2017 ver 2.1.5	- servo angle configuration in range (-50..+50)*[0.1ms] with step 1 								
; 											
;----------------------------------------
; configuration guide
; short press the [PROG] button moves servo to opposite position
; hold the [PROG] button  for 2.5s - 5s PROG_LED turns on 
; when button released  - device enters RANGE configuration mode
;	hold the [PROG] button  longer than 5s - PROG_LED turns off
;	when button released - device enters ADDRESS programming mode
; 
;	In ADDRESS programming mode PROG_LED blinks slow with 1s period and 50% duty cycle.
;		send accessory move command from command station to store new address value, OR 
;		press simultaneous UP & DOWN buttons to set default address (1) 
;		both above actions or short press the [PROG] button exits from programming mode. 
;
;	In RANGE programming mode PROG_LED gives 2 short flashes followed by pause in 1s period
;		use [UP], [DOWN] buttons to change maximum servo angle
;		press simultaneous UP & DOWN buttons to set central position
;		short press [PROG] button to save RANGE value and enter SPEED programming mode
;	In SPEED programming mode PROG_LED gives 3 short flashes followed by pause in 1s period
;		use [UP], [DOWN] buttons to change servo speed
;		press simultaneous UP & DOWN buttons to set default (maximum) speed
;		short press [PROG] button to save SPEED value and exit programming mode
; 

 
;---------------------------------------- 
; ----- Definitions

#define		__VERDAY		0x11
#define		__VERMONTH	0x07
#define		__VERYEAR		0x17
#define		__VERNUM		D'1'



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


	__CONFIG  _BODEN_ON & _CP_OFF & _WDT_OFF & _MCLRE_OFF & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT 

; --- Constant values

FXTAL		equ	D'4000000'		; internal oscilator

; when OSCCAL production calibration is lost, it can be set manually
;#define OSCCAL_VALUE 0x34

GP_TRIS			equ 0x1C			; GP2,GP3: inputs, GP4: analog input (bit=1 input; 0-output)
ANSEL_INI		equ 0x68 ;28     ; GP4/AN3 pin - as analog input  ADCS2:0= 110 -> Tosc/64 = 16us conversion time
ADCON0_INI	equ 0x0C      ; AN3 connected to Sample&Hold , VDD - as voltage reference, result Left justified
WPU_INI			equ 0x23			; Weak pull-up enable only on outputs GP0,GP1,GP5, (turned off when output)
PIE1_INI   	equ	0x41			; interrupt TMR1 (bit0), ADIE(bit6) 
OPTION_INI	equ	0x88			; Option register: no pull-up, falling GP2, no prescaler, wdt 1:1
INTC_INI		equ	0xD0			; GIE(bit7), PEIE enable(bit6),INTE enable(bit4), 

RANGE_MAX   	equ d'50'
RANGE_DEFAULT	equ d'0'
RANGE_MIN			equ 0xCE  ;-50
RANGE_STEP		equ d'1'
; pulse width = (150 +-Range) * 10 *1us = 1.5ms +-0.5ms =  (1ms .. 2ms)
SPEED_MAX 		equ d'1'
SPEED_DEFAULT	equ d'1'
SPEED_MIN 		equ d'4'
SPEED_STEP		equ d'1'




#define		RELE1A	GPIO,0			; Rele output straight - RED Led / Programming mode
#define		RELE1B	GPIO,1			; Rele output diverge -  GREEN Led
#define		DCCIN	  GPIO,2			; DCC input pin
#define		SWITCH	GPIO,3			; Move/Programm switch (GPIO,3 only input)
#define		LED			GPIO,0			; Prog LED on RELE1A output
#define		KBD			GPIO,4			; AN3
#define		SERVO		GPIO,5			; Servo output

REACH_PULS	equ	0x04					; aditional pulses when reached position

; threshold voltage levels corresponding to the button held down
THR_K1	equ	d'190'   
THR_K2	equ	d'150'
THR_K12	equ	d'115'
THR_KERR	equ d'90'
DEBOUNCE_COUNT	equ	0x03  ;bit
DEBOUNCE_NOKEY_COUNT equ 0x02; bit


; --- EEPROM Section

#define		EE_INI		0x00

EE_ADDR1H	equ	EE_INI+0x00
EE_ADDR1L	equ	EE_INI+0x01
EE_RANGE	equ	EE_INI+0x02
EE_SPEED	equ	EE_INI+0x03

EE_OUT		equ	EE_INI+0x7F		; saved outputs


; ----- Variables

; --- Internal RAM Section

#define		RAMINI0		0x020		; 64 bytes of RAM

INT_W			equ	RAMINI0+0x00		; interrupt context registers
INT_STAT	equ	RAMINI0+0x01

SHIFT0		equ	RAMINI0+0x02
DCCDATA1	equ	RAMINI0+0x03  ; DCC data buffer for incoming data
DCCDATA2	equ	RAMINI0+0x04
DCCDATA3	equ	RAMINI0+0x05
DCCDATA4	equ	RAMINI0+0x06

PREAMBLE	equ	RAMINI0+0x08
DCCSTATE	equ	RAMINI0+0x09
DCCBYTE		equ	RAMINI0+0x0A

EEDATA0		equ	RAMINI0+0x0B		; EEPROM shadow variables
EEADR0		equ	RAMINI0+0x0C


SRVADRH1	equ	RAMINI0+0x10		; servo address
SRVADRL1	equ	RAMINI0+0x11		; 
RANGE			equ	RAMINI0+0x12		; servo range
SPEED			equ	RAMINI0+0x13		; servo speed

PULSEH		equ	RAMINI0+0x14   ; calculated PWM pulse width = 100 * RANGE1
PULSEL		equ	RAMINI0+0x15

STATE							equ	RAMINI0+0x1F		; state of decoder (position,moving,programming)
SRV1CNT						equ	RAMINI0+0x20		; speed divider counter
CURRENT_PULSE			equ	RAMINI0+0x21		; current pulse duration
SERVOSTATE				equ	RAMINI0+0x22	; current state
POSITION					equ	RAMINI0+0x23		; current position flags ( inclination direction)
PROGRAMMING_PHASE	equ	RAMINI0+0x24	; current programming phase


SPACE_H			equ	RAMINI0+0x26		; spacing duration
SPACE_L			equ	RAMINI0+0x27
RELEPULSE		equ	RAMINI0+0x28		; bistable relay pulse

DEBOUNCE_PROG_CNT	equ	RAMINI0+0x2A	; PROG key debouncing counter for long periods
DEBOUNCE_PROG_REG	equ	RAMINI0+0x2B	; PROG key debouncing shift register for short periods
TARGET_PULSE 			equ	RAMINI0+0x2C  ; target pulse width calculated for position and range
FLSH_PRE					equ	RAMINI0+0x2D		; flash prescaler
FLSH1							equ	RAMINI0+0x2E		; flash sequence
FLSH2							equ	RAMINI0+0x2F		; 

FLAGS			equ	RAMINI0+0x30

COUNT			equ	RAMINI0+0x32
TEMPL			equ	RAMINI0+0x33
TEMPH			equ	RAMINI0+0x34

DCCDATABF1		equ	RAMINI0+0x35   ; DCC data buffer for decoding
DCCDATABF2		equ	RAMINI0+0x36
DCCDATABF3		equ	RAMINI0+0x37
DCCDATABF4		equ	RAMINI0+0x38
KEY_ADC		equ	RAMINI0+0x39

DEB_AKEY_NO	equ	RAMINI0+0x3A  ;analog key debouncing
DEB_AKEY_1	equ	RAMINI0+0x3B
DEB_AKEY_2	equ	RAMINI0+0x3C
DEB_AKEY_12	equ	RAMINI0+0x3D
AKEY_STATE 	equ	RAMINI0+0x3E

EEPTR		equ	RAMINI0+0x3F		; Page register


; --- Flags
#define	MOVING_STATE			STATE,1
#define	PROGRAMMING_STATE	STATE,2
#define	ADR_PROG_STATE		STATE,3
#define	REACHED_STATE			STATE,4

;;  analog keyboard 
#define	AKEY_UP			AKEY_STATE,0
#define	AKEY_DOWN		AKEY_STATE,1
#define	AKEY_UPDOWN	AKEY_STATE,2


#define		NEW_PACKET	FLAGS,0		; New 3 byte packet received
#define 	END_PULSE		FLAGS,1		; time to check PROG SWITCh
#define   DO_CHANGE_POS  FLAGS,2;  ; Move Servo to opposite position 
#define		DCC4BYTE		FLAGS,3		; DCC command 4 bytes
#define		DO_PULSE		FLAGS,5		; do pulse, TMR1 end
#define		DO_ADC			FLAGS,6		; ADC conversion end
#define		RESET_FLG		FLAGS,7		; reset packet


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
		bcf		SERVO			; yes, clear pulse			;+3
		bcf		PIR1,TMR1IF							;+4
		bsf		DO_PULSE							;+5

Int_DCC:
		btfss	INTCON,INTF		; RB0 interrupt?			;+4,	+6
		goto	Int_ADC					;+5,6,	+7,8								;+5,6,	+7,8

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
		swapf	INT_STAT,w		; restore context registers		;24
		movwf	STATUS								;25
		swapf	INT_W,f								;26
		swapf	INT_W,w								;27
		retfie									;28,29
		
Int_ADC:
		btfss	PIR1,ADIF		; end of ACD conversion
		goto	EndInt
		bcf	PIR1,ADIF   ; clear interrupt flag
		movf ADRESH,w
		movwf KEY_ADC
		; end of acd conversion routine    
		bsf DO_ADC		
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
		clrf	DCCDATA4			;					;16
		goto	EndHighHalf		;					;17,18


ReadBit:
		bsf		STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf		STATUS,C							;14
		rlf		SHIFT0,f		; receiver shift register		;15
		incf	DCCSTATE,f		;					;16
		goto	EndHighHalf		;					;17,18
			
ReadLastBit:
		bsf		STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf		STATUS,C							;14
		rlf		SHIFT0,f		; receiver shift register		;15
		incf	DCCBYTE,w							;16
		addwf	DCCSTATE,f							;17
		goto	EndHighHalf		;					;18,19

EndByte1:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DCCDATA1			;				;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20

EndByte2:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DCCDATA2			;					;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20


EndByte3:
		btfss	INTCON,T0IF		; End bit=1, end of packet		;12
		goto	EndByte3x		;					;13,14
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DCCDATA3			;					;17
		incf	DCCBYTE,f		;					;18
		bsf		DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21
EndByte3x:
		clrf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DCCDATA3			;					;17
		bsf		NEW_PACKET		;					;18
		bcf		DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21

EndByte4:
		clrf	DCCSTATE		;					;12
		btfsc	INTCON,T0IF		; End bit=1, end of packet		;13
		goto	EndInt			; End bit=0, invalid packet		;14,15
		movf	SHIFT0,w		;					;15
		movwf	DCCDATA4			;					;16
		bsf		NEW_PACKET		;					;17
		goto	EndHighHalf		;					;18,19


; ----------------------------------------------------------------------

; ----- Initialization

INIT:
		clrf	GPIO
		movlw	0x07
		movwf	CMCON			; set GP2:0 to digital I/O
		movlw	ADCON0_INI
		movwf	ADCON0   
		bsf	STATUS,RP0		; bank 1
		movlw	ANSEL_INI
		movwf	ANSEL     ; PIC12F675  Analog Select Register
		movlw	GP_TRIS
		movwf	TRISIO
#ifdef	OSCCAL_VALUE    
		movlw OSCCAL_VALUE	; set OSCCAL manually 
#else
		call	0x3FF			; get factory calibrated OSCCAL from last address of Flash
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
		movwf	CURRENT_PULSE
		call	LoadCV			; Load CV in SFR
		call	LoadOutputs		; set servo to last position


; ----------------------------------------------------------------------


MainLoop:
		btfsc	NEW_PACKET		; new packet?
		call	Decode			; yes, decode

		btfsc	DO_PULSE		; end of servo pulse?
		call	DoServo			; yes, next pulse
		clrwdt
		btfss	END_PULSE		; every 10ms END_PULSE
		goto	MainLoop
		bcf END_PULSE
		call CheckKey
		goto	MainLoop


DoServo:
		bcf	DO_PULSE
		btfsc	INTCON,INTE		; disabled interrupts?
		goto	DoServoJump
		clrf	DCCSTATE		; yes, clear for decoding
		bsf	INTCON,INTE		; re-enable interrupts
DoServoJump:
		movf	SERVOSTATE,w
		andlw	0x03
		addwf	PCL,f

		goto	PulseServo1
		goto	PulseServo2
		goto	Spacing
		clrf	SERVOSTATE			; prevents erroneus contents


	if ($ > d'255') 
		ERROR "  Tables exceded page 0.   If it works, why do you change it?   "
	endif

; ----------------------------------------------------------------------

PulseServo1:
		clrf	SPACE_H			; init spacing value (10ms)
		clrf	SPACE_L

		movf	CURRENT_PULSE,w		; 200: 2ms, 100: 1ms
		call	PulseServo

		btfsc	MOVING_STATE
		bcf	INTCON,INTE		; disable DCC interrupts for time accuracy

		btfsc	MOVING_STATE
		bsf	SERVO

		bsf	T1CON,TMR1ON		; run timer 1
		incf	SERVOSTATE,f

		movf	RELEPULSE,w		; relay pulse
		btfss	STATUS,Z
		decfsz	RELEPULSE,f
		goto	EndSpacing
		bcf	RELE1A
		bcf	RELE1B
		goto	EndSpacing


PulseServo2:
		movlw	d'150'			; 200: 2ms, 100: 1ms, 150: 1.5ms (center)
		call	PulseServo		; dummy pulse
		bsf	T1CON,TMR1ON		; run timer 1
		incf	SERVOSTATE,f
		goto	EndServo1




Spacing:
		bcf	T1CON,TMR1ON
		movf	SPACE_H,w		; pulses length
		movwf	TMR1H
		movf	SPACE_L,w
		movwf	TMR1L
	
	; old 20ms time ( 0x10000 - 0x4E20 = 0xB1E0 )
	; new 10ms  ( 0x10000 - 0x2710 = 0xD8F0 )
		movlw	0xF0 			
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
		movlw	0xD8
		addwf	TMR1H,f		
		bsf	T1CON,TMR1ON		; run timer 1
		clrf	SERVOSTATE
		bsf END_PULSE
		return

CalculatePulse:		
		; W*10
		bcf	T1CON,TMR1ON		; stop timer 1
		clrf	TMR1H			; do PULSE x10
		movwf	TEMPL
		movwf	TMR1L
		rlf	TMR1L,f
		rlf	TMR1H,f
		rlf	TMR1L,f			; x4
		rlf	TMR1H,f
		rlf	TMR1L,f			; x8
		rlf	TMR1H,f
		movlw	0xF8
		andwf	TMR1L,f
		movf	TEMPL,w			; x8 + x1 = x9
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
		addwf	TMR1L,f			; x9 + x1 = x10
		btfsc	STATUS,C
		incf	TMR1H,f
	return
	
PulseServo:
		call CalculatePulse
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


EndSpacing:					; every 10ms
		return
		

;handling bidirectional range configuration (-50,50) - EndServo1 replacement
EndServo1:
		btfss	MOVING_STATE		; moving servo?
		return				; no
		btfss	POSITION,0	
		goto EndServo1a		
		comf	RANGE,w		; range (negative)
		addlw	0x01
		addlw	d'150'
		movwf TARGET_PULSE
		goto EndServo1x
EndServo1a:
		movf	RANGE,w
		addlw	d'150'		
		movwf TARGET_PULSE
EndServo1x:	
		btfsc	REACHED_STATE
		goto	EndServo1Reached  
		decfsz	SRV1CNT,f		; speed
		goto	EndServo1Cmp
		movf	SPEED,w
		movwf	SRV1CNT
    ; change CURRENT_PULSE
		movf  CURRENT_PULSE,w  
		subwf TARGET_PULSE,w  ;f-W -> W ; C=0 for negative; C=1 for positive and Zero
		btfsc STATUS,Z
		goto EndServo1Equ
		btfsc STATUS,C
		incf CURRENT_PULSE,f  ; current>target
		btfss STATUS,C
		decf CURRENT_PULSE,f		
EndServo1Cmp:
		movf  CURRENT_PULSE,w 
		xorwf	TARGET_PULSE,w
		btfss	STATUS,Z
		return
EndServo1Equ:		
		movlw	REACH_PULS
		movwf	SRV1CNT
		bsf	REACHED_STATE
		return
EndServo1Reached:		
		decfsz	SRV1CNT,f		; additional pulses
		return
		movlw 0x01
		btfsc DO_CHANGE_POS
		xorwf	POSITION,f		; save new output state
		goto EndServoSave
		
EndServoSave:
		bcf DO_CHANGE_POS
		bcf	REACHED_STATE
		bcf	MOVING_STATE
		btfsc PROGRAMMING_STATE
		return					; 
		movlw	d'10'			; 10 * 10ms relay pulse
		movwf	RELEPULSE
		btfss	POSITION,0
		bsf	RELE1A
		btfsc	POSITION,0
		bsf	RELE1B

		movf	POSITION,w		; save new output state
		xorlw	0x01	
		movwf	EEDATA0
		movlw	EE_OUT
		goto	SetParm



CheckKey:				; every 10ms
		btfsc	SWITCH			; check program switch 
		goto	ReadInputSwitch
		decf	DEBOUNCE_PROG_CNT,f		; *** for 2,5s
		decfsz DEBOUNCE_PROG_CNT,f		; pressed, wait debounce time (2,5s)
		return
		goto	ProgMain		; enter programming mode
ReadInputSwitch:
		movf	DEBOUNCE_PROG_CNT,w		; short pressed?
		btfsc	STATUS,Z
		return				; no
Key_Change:
		; servo move
		bsf DO_CHANGE_POS
		bsf	MOVING_STATE
		movf	SPEED,w
		movwf	SRV1CNT
		clrf	DEBOUNCE_PROG_CNT
		return



; ----------------------------------------------------------------------


ProgMain:
		bsf PROGRAMMING_STATE
		bsf ADCON0,ADON     ; enable ADC for analog keyboard 
		bsf	LED			; LED on
		movlw 0x26     ;; 38*65ms = 2.5s  to enter address programming mode
		movwf DEBOUNCE_PROG_CNT ; in next 2.5s enter address programming mode.
ProgStep1: 
		clrwdt  
		btfsc SWITCH				; wait to release 
		goto ProgServoRange	
		btfss	DO_PULSE			;every 65ms
		goto ProgStep1 
		bcf DO_PULSE 
		decfsz	DEBOUNCE_PROG_CNT,f  
		goto ProgStep1
		; address programming mode
ProgAddress:
		clrwdt
		bcf LED   ; again LED is off  
		btfss	SWITCH			; wait to release
		goto	ProgAddress
		bsf ADCON0,GO_NOT_DONE  ; start conversion
		bsf ADR_PROG_STATE    
		clrf EEPTR
		bsf	FLSH_PRE,1 ;
		call	SetFlash
		goto ProgLoop
ProgServoRange:  
		bsf ADCON0,GO_NOT_DONE  ; start conversion  
		clrf	EEPTR
		bsf EEPTR,1  ; Range programming
		bsf	FLSH_PRE,1
		call	SetFlash
ProgLoop:
		clrwdt
		btfsc	DO_PULSE		
		call	DoServo
		btfss END_PULSE  ; every 10ms
		goto	ProgKey
		bcf END_PULSE
		decfsz	FLSH_PRE,f		; prescaler flash
		goto	ProgADC
		bsf	FLSH_PRE,4   ; 0x16
		bcf	STATUS,C
		btfsc	FLSH1,7
		bsf	STATUS,C
		rlf	FLSH2,f
		rlf	FLSH1,f
		btfss	STATUS,C
		bcf	LED
		btfsc	STATUS,C
		bsf	LED
ProgADC:    
    ; Adc keyboard  - every 10ms
    btfss DO_ADC
    goto ProgKey
    bcf DO_ADC
    ;; compare adc value with thresholds
    movlw THR_K1
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyReleased   ;KEY_ACC>THR_K1
    clrf DEB_AKEY_NO
    movlw THR_K2
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyUp   ;KEY_ACC>THR_K2
    movlw THR_K12
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyDown   ;KEY_ACC>THR_K12
    movlw THR_KERR
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyUpDown   ;KEY_ACC>THR_KERR
KeyERR:    
    clrf DEB_AKEY_NO
    clrf DEB_AKEY_1
    clrf DEB_AKEY_2
    clrf DEB_AKEY_12
    bcf AKEY_UP
    bcf AKEY_DOWN
    bcf AKEY_UPDOWN
    goto KeyEnd
KeyReleased:
    bsf STATUS,C
    rlf DEB_AKEY_NO
    btfss DEB_AKEY_NO,DEBOUNCE_NOKEY_COUNT
    goto KeyEnd
    btfsc DEB_AKEY_1,DEBOUNCE_COUNT
    bsf AKEY_UP
    btfsc DEB_AKEY_2,DEBOUNCE_COUNT
    bsf AKEY_DOWN
    btfsc DEB_AKEY_12,DEBOUNCE_COUNT
    bsf AKEY_UPDOWN
    clrf DEB_AKEY_NO
    clrf DEB_AKEY_1
    clrf DEB_AKEY_2
    clrf DEB_AKEY_12
    goto KeyEnd
KeyUp:  
    btfsc DEB_AKEY_2,DEBOUNCE_COUNT   ;manage releasing keys
    goto KeyEnd
    btfsc DEB_AKEY_12,DEBOUNCE_COUNT
    goto KeyEnd
    bsf STATUS,C      ; key still pressed
		rlf DEB_AKEY_1
    goto KeyEnd
KeyDown:
    btfsc DEB_AKEY_12,DEBOUNCE_COUNT  ;manage releasing keys 
    goto KeyEnd
    clrf DEB_AKEY_1
    bsf STATUS,C        ; key still pressed
    rlf DEB_AKEY_2     
    goto KeyEnd
KeyUpDown:
    clrf DEB_AKEY_1
    clrf DEB_AKEY_2
    bsf STATUS,C
    rlf DEB_AKEY_12
    goto KeyEnd
KeyEnd:    
    bsf ADCON0,GO_NOT_DONE  ; start conversion
ProgKey:
		bcf	STATUS,C		; check key
		btfsc	SWITCH
		bsf	STATUS,C
		rlf	DEBOUNCE_PROG_REG,w
		movwf	DEBOUNCE_PROG_REG
		xorlw	0xC0
		btfss	STATUS,Z
		goto	ProgNoKey
		movf EEPTR,w
    btfsc STATUS,Z
    goto EndProg   ; exit from address programming
		btfss	EEPTR,1
		incf	EEPTR,f
		incf	EEPTR,f
		call	SetFlash
		btfss	EEPTR,2
		goto	ProgLoop
EndProg:
		bcf ADCON0,GO_NOT_DONE  ; stop pending ADC conversion
		bcf	LED
		bcf ADR_PROG_STATE
    bcf PROGRAMMING_STATE
    bcf ADCON0,ADON     ; dissable ADC converter 
		bsf	INTCON,INTE		; enable interrupts
		return

ProgNoKey:
    btfsc	EEPTR,1
    goto ProgAKeyRangeSpeed
    ;analog keyboard during address programming
    bcf AKEY_DOWN  ; action not defined 
    bcf AKEY_UP    ; action not defined
    btfss AKEY_UPDOWN
    goto ProgAkeyAddressEnd
    ; up and down pressed during address programming
    ; reset address
    bcf AKEY_UPDOWN
    movlw 0x81
    movwf DCCDATABF1
    movlw 0xF8
    movwf DCCDATABF2
    goto	ProgSetAddr		; address
ProgAkeyAddressEnd:
		btfss	NEW_PACKET		; new packet?
		goto	ProgLoop

DecodeProg:
    ; copy DATA to decoder buffer
    movf DCCDATA1,w
    movwf DCCDATABF1
    movf DCCDATA2,w
    movwf DCCDATABF2
    movf DCCDATA3,w
    movwf DCCDATABF3
    movf DCCDATA4,w
    movwf DCCDATABF4
		
		bcf	NEW_PACKET		; prepare for next packet
		bcf	INTCON,INTE		; disable interrupts for more speed

		movf	DCCDATABF1,w			; exclusive or check
		xorwf	DCCDATABF2,w
		xorwf	DCCDATABF3,w
		xorwf	DCCDATABF4,w

		btfss	STATUS,Z		; valid packet?
		goto	ExitDecodeProg		; no, return

		movf	DCCDATABF1,w			;'10AAAAAA'  '1AAADxxx' AAA:111 D:1 activate
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfsc	DCCDATABF2,7
		btfss	STATUS,Z
		goto	ExitDecodeProg
		btfss	DCCDATABF2,3			; activate?
		goto	ExitDecodeProg
		goto	ProgSetAddr		; address

		
ProgAKeyRangeSpeed:
    btfss	EEPTR,0
    goto ProgAKeyRange
    ; Prog Speed
    btfss AKEY_UP
    goto  ProgAKeySpeedDown
		bcf AKEY_UP
    ; keyUp
    movlw SPEED_MAX    ;it is the max speed, the more the slowly
    subwf SPEED,w  
    btfsc STATUS,Z 
    goto ProgAKeySpeedEnd  
    movlw SPEED_STEP     ;decrement
		subwf SPEED,w
    goto ProgAKeySetSpeedValue    
ProgAKeySpeedDown:   
    btfss AKEY_DOWN
    goto  ProgAKeySpeedUpDown
    bcf AKEY_DOWN
		movlw SPEED_MIN  
    subwf SPEED,w  
    btfsc STATUS,Z 
    goto ProgAKeySpeedEnd  
    movlw SPEED_STEP   ; increment
    addwf SPEED,w
    goto ProgAKeySetSpeedValue      
ProgAKeySpeedUpDown:   
    btfss AKEY_UPDOWN
    goto  ProgAKeySpeedEnd
		bcf AKEY_UPDOWN
    movlw SPEED_DEFAULT  
ProgAKeySetSpeedValue:    
    movwf SPEED
    movwf	EEDATA0
    movf	EEPTR,w
		call	SetParm
		bsf DO_CHANGE_POS
		call Key_Change
ProgAKeySpeedEnd:    
    bcf AKEY_UP
    bcf AKEY_DOWN
    bcf AKEY_UPDOWN
    goto  ProgLoop
ProgAKeyRange:
    btfss AKEY_UP
    goto  ProgAKeyRangeDown
    ; keyUp
		bcf AKEY_UP
    movlw RANGE_MAX    
    subwf RANGE,w
    btfsc STATUS,Z 
    goto ProgAKeyRangeEnd  
    movlw RANGE_STEP     ;
    addwf RANGE,w
    goto ProgAKeySetRangeValue
ProgAKeyRangeDown:   
    btfss AKEY_DOWN
    goto  ProgAKeyRangeUpDown
    bcf AKEY_DOWN
		movlw RANGE_MIN   ;  min
    subwf RANGE,w  
    btfsc STATUS,Z 
    goto ProgAKeyRangeEnd  
    movlw RANGE_STEP   
    subwf RANGE,w
    goto ProgAKeySetRangeValue
ProgAKeyRangeUpDown:   
    btfss AKEY_UPDOWN
    goto  ProgAKeyRangeEnd
    bcf AKEY_UPDOWN
		movlw RANGE_DEFAULT  ;  
ProgAKeySetRangeValue:    
    movwf RANGE
    movwf	EEDATA0
    movf	EEPTR,w
		call	SetParm
		call SetServoPositionNoChange	
	
ProgAKeyRangeEnd:    
    bcf AKEY_UP
    bcf AKEY_DOWN
    bcf AKEY_UPDOWN
    goto  ProgLoop
    
ProgSetAddr:
		movf	DCCDATABF1,w
		movwf	EEDATA0
		movlw	EE_ADDR1H
		call	SetParm			; save address
		movf	DCCDATABF2,w
    andlw 0xFE    ; make sure, address not depending on coil select bit D in '1AAA1CCD' 
		movwf	EEDATA0
		movlw	EE_ADDR1L
		call	SetParm			; save address
		call	LoadCV
		; set Position depending on bit D
		bcf POSITION,0
		btfsc DCCDATABF2,0 
		bsf POSITION,0
		bsf DO_CHANGE_POS
		bsf	MOVING_STATE		; move to test
		movlw d'150'
		movwf CURRENT_PULSE
		movf	RANGE,w
		movwf	SRV1CNT
		goto	EndProg

ExitDecodeProg:
		bsf	INTCON,INTE		; enable interrupts
		goto	ProgLoop



SetFlash:
		btfss	EEPTR,1
		goto	FlashLED
		movlw	b'10100000'		 ; RANGE  
		btfsc	EEPTR,0
		movlw	b'10101000'    ; SPEED
		movwf	FLSH1
		movwf	FLSH2
		return
FlashLED:
		movlw	b'11110000'   ; addres programming
		movwf	FLSH1
		movwf	FLSH2
		return


; ----------------------------------------------------------------------

Decode:
		movf DCCDATA1,w
    movwf DCCDATABF1
    movf DCCDATA2,w
    movwf DCCDATABF2
    movf DCCDATA3,w
    movwf DCCDATABF3
    movf DCCDATA4,w
    movwf DCCDATABF4
		
		
		bcf	NEW_PACKET		; prepare for next packet
		bcf	INTCON,INTE		; disable interrupts for more speed

		movf	DCCDATABF1,w			; exclusive or check
		xorwf	DCCDATABF2,w
		xorwf	DCCDATABF3,w
		xorwf	DCCDATABF4,w

		btfss	STATUS,Z		; valid packet?
		goto	ExitDecode		; no, return

; 'AAAAAAAA''DDDDDDDD''EEEEEEEE'		; 3 byte packet
;   DATA1     DATA2     DATA3

		movf	DCCDATABF1,w			; address = '00000000' ?
		btfsc	STATUS,Z
		goto	Broadcast
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfss	STATUS,Z
		goto	ExitDecode
		goto	Accessory

Accessory:
		bcf	RESET_FLG

AccSrv1:
		movf	SRVADRH1,w		; '10AAAAAA' of servo 1
		xorwf	DCCDATABF1,w
		btfss	STATUS,Z
		goto	AccSrv2
		movf	SRVADRL1,w		; '1AAACDDD'
		xorwf	DCCDATABF2,w
		btfss	STATUS,Z
		goto	AccSrv1B
		btfss	POSITION,0		; move servo
		bsf		MOVING_STATE
		bsf 	DO_CHANGE_POS
		goto	AccSrv1C
AccSrv1B:
		xorlw	0x01			; other position?
		btfss	STATUS,Z
		goto	AccSrv2
		btfsc	POSITION,0		; move servo
		bsf		MOVING_STATE
		bsf 	DO_CHANGE_POS
AccSrv1C:
		movf	SPEED,w		; set speed
		movwf	SRV1CNT

AccSrv2:
		goto	ExitFunction


ExitFunction:
		bcf	RESET_FLG
		bsf	INTCON,INTE		; enable DCC interrupts
		return


; -----------------------------------------------------------------------

Broadcast:
		movf	DCCDATABF2,w			; reset packet?
		btfss	STATUS,Z
		goto	ExitDecode
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
		movlw	SPEED			; last CV to read
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
		movf	RANGE,w		; calculate pulse for position
		btfsc	POSITION,0		; ***************
		xorlw	0xFF
		btfsc	POSITION,0		; ***************
		addlw	0x01
		addlw	d'150'
		movwf	CURRENT_PULSE
		movlw	0x20			; pulses for setting position
		movwf	SRV1CNT
		bsf	REACHED_STATE		; do move to reached position
		bsf DO_CHANGE_POS
		bsf	MOVING_STATE
		return

SetServoPositionNoChange:
		bcf DO_CHANGE_POS
		movf	RANGE,w		; calculate pulse for position
		btfss	POSITION,0		; ***************
		xorlw	0xFF
		btfss	POSITION,0		; ***************
		addlw	0x01
		addlw	d'150'
		movwf	CURRENT_PULSE
		movlw	0x20			; pulses for setting position
		movwf	SRV1CNT
		bsf	REACHED_STATE		; do move to reached position
		bsf	MOVING_STATE
		bcf DO_CHANGE_POS
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
