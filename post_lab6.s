;Dispositivo:	PIC16F887
;Autor:		Carlos Búcaro
;Compilador:	pic-as (v2.30), MPLABX V5.40
;Programa:	Contador de 8 bits con timer 1 y led interminente en el puerto E
;Hardware
;		LeD´s en el puerto E0 y salidas en el puerto C y contador binario en el
;		puerto A
;Creado: 5 de marzo, 2022
;Última modificación: 5 de marzo, 2022

PROCESSOR   16F887
#include    <xc.inc>
    
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

RESET_TMR1 MACRO TMR1_H, TMR1_L
    MOVLW   TMR1_H	    ; Literal a guardar en TMR1H
    MOVWF   TMR1H	    ; Guardamos literal en TMR1H
    MOVLW   TMR1_L	    ; Literal a guardar en TMR1L
    MOVWF   TMR1L	    ; Guardamos literal en TMR1L
    BCF	    TMR1IF	    ; Limpiamos bandera de int. TMR1
    ENDM
RESET_TMR0 MACRO TMR_VAR
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   TMR_VAR
    MOVWF   TMR0	    ; configuramos tiempo de retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    ENDM
    
PSECT udata_shr
    W_TEMP:		DS  1
    STATUS_TEMP:	DS  1
    LIM:		DS  1
PSECT udata_bank0
    UNIT:		DS  1
    DECS:		DS  1
    LIMU:		DS  1
    LIMD:		DS  1
    DISPLAY:		DS  2
    BANDERAS:		DS  1
    
    
;--------------VECTOR DE RESET----------------------------
PSECT resVect, class=CODE, abs, delta=2
ORG 00h			    ; posición 0000h para el reset

resetVec:
    PAGESEL MAIN	    ; Cambio de pagina
    GOTO    MAIN

;------------ VECTOR DE INTERRUPCIONES-------------------
PSECT intVect, class=CODE, abs, delta=2
ORG 04h
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
   BTFSC    T0IF
   CALL	    INT_TMR0
   BTFSC    TMR1IF
   CALL	    INT_TMR1	 
   BTFSC    TMR2IF
   CALL	    INT_TMR2
     
POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal
    
;----------------Código principal------------------------
ORG 200h
TABLA:
    CLRF    PCLATH		; Limpiamos registro PCLATH
    BSF	    PCLATH, 1		; Posicionamos el PC en dirección 02xxh
    ANDLW   0x0F		; no saltar más del tamaño de la tabla
    ADDWF   PCL
    RETLW   00111111B	;0
    RETLW   00000110B	;1
    RETLW   01011011B	;2
    RETLW   01001111B	;3
    RETLW   01100110B	;4
    RETLW   01101101B	;5
    RETLW   01111101B	;6
    RETLW   00000111B	;7
    RETLW   01111111B	;8
    RETLW   01101111B	;9
MAIN:
    MOVLW   10
    MOVWF   LIMU
    MOVLW   10
    MOVLW   LIMD
    MOVLW   2
    MOVWF   LIM
    CALL    CONFIG_IO
    CALL    CONFIG_RELOJ
    CALL    CONFIG_TMR1
    CALL    CONFIG_TMR2
    CALL    CONFIG_INT_TMR
;----------------Loop------------------------------------
LOOP:
    MOVF    PORTA, W
    MOVWF   UNIT
    CALL    SET_DISPLAY
    GOTO	LOOP
;------------Módulo de configuraciones-------------------
    
CONFIG_IO:
    BANKSEL	ANSEL
    CLRF	ANSEL
    CLRF	ANSELH
    BANKSEL	TRISA
    CLRF	TRISA ;	Establecer el puerto A como salida
    BANKSEL	PORTA 
    CLRF	PORTA ; Limpiar el puerto A
    BANKSEL	TRISC ;	Establecer el puerto B como salida
    CLRF	TRISC
    BANKSEL	PORTC ;limpiar el puerto C
    CLRF	PORTC
    BANKSEL	TRISD
    CLRF	TRISD ; Establcer el puerto D como salida
    BANKSEL	PORTD
    CLRF	PORTD ; Limpiar el puerto D
    BANKSEL	TRISE
    CLRF	TRISE ; Establecer el puerto E como salida
    BANKSEL	PORTE
    CLRF	PORTE ; Limpiar el puerto E
    CLRF	BANDERAS    
    RETURN
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; cambiamos a banco 1
    BSF	    OSCCON, 0	    ; SCS -> 1, Usamos reloj interno
    BCF	    OSCCON, 6
    BSF	    OSCCON, 5
    BCF	    OSCCON, 4	    ; IRCF<2:0> -> 010 250KHz
    RETURN
CONFIG_TMR0:
    BANKSEL OPTION_REG		; cambiamos de banco
    BCF	    T0CS		; TMR0 como temporizador
    BCF	    PSA			; prescaler a TMR0
    BCF	    PS2
    BSF	    PS1
    BCF	    PS0			; PS<2:0> -> 010 prescaler 1 : 8
    RESET_TMR0 248		; Reiniciamos TMR0 para 1ms
    RETURN 
CONFIG_TMR1:
    BANKSEL T1CON	    ; Cambiamos a banco 00
    BCF	    TMR1GE	    ; TMR1 siempre cuenta
    BSF	    T1CKPS1	    ; prescaler 1:8
    BSF	    T1CKPS0
    BCF	    T1OSCEN	    ; LP deshabilitado
    BCF	    TMR1CS	    ; Reloj interno
    BSF	    TMR1ON	    ; Prendemos TMR1
    
    RESET_TMR1 0xE1, 0x7C   ; Reiniciamos TMR1 para 1000ms
    RETURN
CONFIG_TMR2:
    BANKSEL PR2		    ; Cambiamos a banco 01
    MOVLW   122		    ; Valor para interrupciones cada 500ms
    MOVWF   PR2		    ; Cargamos litaral a PR2
    
    BANKSEL T2CON	    ; Cambiamos a banco 00
    BSF	    T2CKPS1	    ; prescaler 1:16
    BSF	    T2CKPS0
    
    BSF	    TOUTPS3	    ; postscaler 1:16
    BSF	    TOUTPS2
    BSF	    TOUTPS1
    BSF	    TOUTPS0
    
    BSF	    TMR2ON	    ; prendemos TMR2
    RETURN
    
CONFIG_INT_TMR:
    BANKSEL PIE1	    ; Cambiamos a banco 01
    BSF	    T0IE
    BSF	    TMR1IE	    ; Habilitamos interrupciones de TMR1
    BSF	    TMR2IE	    ; Habilitamos interrupciones de TMR2
    
    BANKSEL INTCON	    ; Cambiamos a banco 00
    BSF	    PEIE	    ; Habilitamos interrupciones de perifericos
    BSF	    GIE		    ; Habilitamos interrupciones
    BCF	    T0IF	    ; Limpiamos bandera de TMR0
    BCF	    TMR1IF	    ; Limpiamos bandera de TMR1
    BCF	    TMR2IF	    ; Limpiamos bandera de TMR2
    RETURN
;----------------Módulo de interrupciones-------------------
INT_TMR0:
    RESET_TMR0	248
    CALL    MOSTRAR_VALORES
    RETURN
INT_TMR1:
    RESET_TMR1 0xE1, 0x7C   ; Reiniciamos TMR1 para 1000ms
    INCF    PORTA	    ; Incremento en PORTA
    DECFSZ  LIMU
    RETURN
    CALL    OBTENER_VALORES
    RETURN
INT_TMR2:
    BCF	    TMR2IF	    ; Limpiamos bandera de interrupcion de TMR2
    INCF    PORTE	    ; Incremento en PORTC
    DECFSZ  LIM
    RETURN
    MOVLW   2
    MOVWF   LIM
    CLRF    PORTE
    RETURN
;---------------Módulo de multiplexado-----------------------
OBTENER_VALORES:
    CLRF    UNIT
    MOVLW   10
    MOVWF   LIMU
    INCF    DECS
    DECFSZ  LIMD
    RETURN
    CLRF    DECS   
    MOVLW   10
    MOVWF   LIMD
SET_DISPLAY:
    MOVF    UNIT, W
    CALL    TABLA
    MOVWF   DISPLAY
    
    MOVF    DECS, W
    CALL    TABLA
    MOVWF   DISPLAY+1
MOSTRAR_VALORES:
    BCF	    PORTD, 0		; Apagamos display de nibble alto
    BCF	    PORTD, 1		; Apagamos display de nibble bajo
    BTFSC   BANDERAS, 0		; Verificamos bandera
    GOTO    DISPLAY_1		;  
    ;GOTO    DISPLAY_0
    DISPLAY_0:			
	MOVF    DISPLAY, W	; Movemos display a W
	MOVWF   PORTC		; Movemos Valor de tabla a PORTC
	BSF	PORTD, 1	; Encendemos display de nibble bajo
	BSF	BANDERAS, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN

    DISPLAY_1:
	MOVF    DISPLAY+1, W	; Movemos display+1 a W
	MOVWF   PORTC		; Movemos Valor de tabla a PORTC
	BSF	PORTD, 0	; Encendemos display de nibble alto
	BCF	BANDERAS, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN
   
 
    
END


