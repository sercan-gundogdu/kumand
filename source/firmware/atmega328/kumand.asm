//-----------------------------------------------------------------------------
//
//
//-----------------------------------------------------------------------------
.include "m328pdef.inc"

//-----------------------------------------------------------------------------
.dseg

// Allocate 32 bytes of memory on SRAM for UART Transmit.
UART_TRANSMIT_BUFFER: .byte 32
// Allocate 32 bytes of memory on SRAM for UART Receive.
UART_RECEIVE_BUFFER:  .byte 32

//-----------------------------------------------------------------------------
.cseg

.org 0x0000 rjmp RESET // Reset Handler
//.org 0x0002 rjmp INT0 // IRQ0
//.org 0x0004 rjmp INT1 // IRQ1
//.org 0x0006 rjmp PCINT0 // PCINT0
//.org 0x0008 rjmp PCINT1 // PCINT1
//.org 0x000A rjmp PCINT2 // PCINT2
//.org 0x000C rjmp WDT // Watchdog Timeout
//.org 0x000E rjmp TIM2_COMPA // Timer2 CompareA
//.org 0x0010 rjmp TIM2_COMPB // Timer2 CompareB
//.org 0x0012 rjmp TIM2_OVF // Timer2 Overflow
//.org 0x0014 rjmp TIM1_CAPT // Timer1 Capture
//.org 0x0016 rjmp TIM1_COMPA // Timer1 CompareA
//.org 0x0018 rjmp TIM1_COMPB // Timer1 CompareB
//.org 0x001A rjmp TIM1_OVF // Timer1 Overflow
//.org 0x001C rjmp TIM0_COMPA // Timer0 CompareA
//.org 0x001E rjmp TIM0_COMPB // Timer0 CompareB
//.org 0x0020 rjmp TIM0_OVF // Timer0 Overflow
//.org 0x0022 rjmp SPI_STC // SPI Transfer Complete
//.org 0x0024 rjmp UART_RXC // UART RX Complete
//.org 0x0026 rjmp UART_UDRE // UART UDR Empty
//.org 0x0028 rjmp UART_TXC // UART TX Complete
//.org 0x002A rjmp ADC // ADC Conversion Complete
//.org 0x002C rjmp EE_RDY // EEPROM Ready
//.org 0x002E rjmp ANA_COMP // Analog Comparator
//.org 0x0030 rjmp TWI // 2-wire Serial
//.org 0x0032 rjmp SPM_RDY // SPM Ready

.equ PORTB_MASK = 0b00111111
.equ PORTC_MASK = 0b00100111
.equ PORTD_MASK = 0b11101100

.equ GET_WLAN_STATUS = 0x02
.equ GET_PC_STATUS = 0x03
.equ SEND_KEY = 0x04
.equ SET_DESTINATION_IP = 0x05
.equ RUN_WPSPBC = 0x0A
.equ GET_DESTINATION_IP = 0x0B

//-----------------------------------------------------------------------------
.macro eepromWrite
  // Load the address of the string to be sent, to r16:r17
  ldi r17, high(@0)
  ldi r16, low(@0)

  // Push the address to stack and call EEPROM_Write function
  push r16
  push r17
  rcall EEPROM_Write
  pop r17
  pop r16

.endmacro

//-----------------------------------------------------------------------------
.macro eepromRead
  // Load the address of the string to be read, to r16:r17
  ldi r17, high(@0)
  ldi r16, low(@0)

  // Push the address to stack and call EEPROM_Read function
  push r16
  push r17
  rcall EEPROM_Read
  pop r17
  pop r16

.endmacro

//-----------------------------------------------------------------------------
RESET:
  // Initialize Stack Pointer
  ldi r16, high(RAMEND)
  out SPH, r16 
  ldi r16, low(RAMEND) 
  out SPL, r16 

  // Initialize GPIO
  rcall GPIO_Init

  // Initialize UART
  rcall UART_Init

  // Set r8 to 0xFF as a refrence
  ldi r16, 0xFF
  mov r8, r16
  clr r16
  
  // Ignore incoming garbage and wait for the response 0x01
  rcall UART_Receive_Byte
  cpi r16, 0x01
  brne PC-2

  // Send destination IP
  rcall IP_SYNC

  rjmp MAIN

//-----------------------------------------------------------------------------
GPIO_Init:
  // Set PORTB pins 0, 1, 2, 3, 4, 5 as INPUT
  ldi r16, (0<<PB0) | (0<<PB1) | (0<<PB2) | (0<<PB3) | (0<<PB4) | (0<<PB5)
  out DDRB, r16
  
  // Internal pull-up for PORTB pins 0, 1, 2, 3, 4, 5
  ldi r16, (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB4) | (1<<PB5)
  out PORTB, r16
  
  // Set PORTC pins 0, 1, 2, 5 as INPUT
  //  3, 4 as OUTPUT
  ldi r16, (0<<PC0) | (0<<PC1) | (0<<PC2) | (1<<PC3) | (1<<PC4) | (0<<PC5)
  out DDRC, r16
  
  // Internal pull-up for PORTC pins 0, 1, 2, 5
  //  Pins 3,4 driven low
  ldi r16, (1<<PC0) | (1<<PC1) | (1<<PC2) | (0<<PC3) | (0<<PC4) | (1<<PC5) 
  out PORTC, r16
  
  // Set PORTD pins 2, 3, 5, 6, 7 as INPUT
  ldi r16, (0<<PD2) | (0<<PD3) | (0<<PD5) | (0<<PD6) | (0<<PD7)
  out DDRD, r16
  
  // Internal pull-up for PORTD pins 2, 3, 4, 5, 6, 7
  ldi r16, (1<<PD2) | (1<<PD3) | (1<<PD5) | (1<<PD6) | (1<<PD7)
  out PORTD, r16
  
  ret

//-----------------------------------------------------------------------------
UART_Init:
  // UBRR0 = 0x003 for 115.2K baudrate at 7.3728MHz.
  // Load 0x003 to UBRR0(USART Baud Rate Register)
  ldi r17, 0x0
  ldi r16, 0x03
  sts UBRR0H, r17
  sts UBRR0L, r16
  
  // Enable Receiver and Transmitter in -
  // UCSR0B(USART Control and Status Register 0B)
  // Normal port operations overriden!
  ldi r16, (1<<RXEN0) | (1<<TXEN0) 
  sts UCSR0B, r16
  
  // Set UCSR0C(USART Control and Status Register 0C) = 0x06
  // Frame format: 8 data bit, 1 stop bit, no parity.
  ldi r16, (0<<USBS0) | (3<<UCSZ00)
  sts UCSR0C, r16
  
  ret

//-----------------------------------------------------------------------------
UART_Transmit_Byte:
  // This function transmits a single byte stored in r16 over UART 
  // Wait for UDRE0 (USART Data Register Empty) bit is set in -
  // USART Control and Status Register
  lds r17, UCSR0A
  sbrs r17, UDRE0
  rjmp UART_Transmit_Byte
  
  // Load byte stored in r16 to buffer and send byte
  // Data will be sent and UDR0(USART I/O Data Register) will be cleared -
  // automatically after load into it
  sts UDR0, r16

UART_Transmit_Byte_Ret: 
  ret

//-----------------------------------------------------------------------------
UART_Transmit_Str:
  // The data being stored in buffer called UART_TRANSMIT_BUFFER will be sent
  // Load buffer address to register X
  ldi XH, high(UART_TRANSMIT_BUFFER)
  ldi XL, low(UART_TRANSMIT_BUFFER)

UART_Transmit_Str_Loop: 
  // Load data stored in buffer into r16 and transmit
  // Termination character is included.
  ld r16, X+
  rcall UART_Transmit_Byte

  // If data is not null-termination byte, continue transmitting
  cpi r16, '\0'
  brne UART_Transmit_Str_Loop

UART_Transmit_Str_Ret:  
  ret

//-----------------------------------------------------------------------------
UART_Receive_Byte:
  // This function receives a single byte over UART and store it in r16
  // Wait for RXC0 (USART Receive Complete) bit is set in -
  // USART Control and Status Register
  lds r17, UCSR0A
  sbrs r17, RXC0
  rjmp UART_Receive_Byte
  
  // Receive data and store it in register r16 
  // UDR0(USART I/O Data Register) will be cleared automatically after -
  // reading from it
  lds r16, UDR0
  
UART_Receive_Byte_Ret:  
  ret

//-----------------------------------------------------------------------------
UART_Receive_Str:
  // The received data will be stored in buffer called UART_RECEIVE_BUFFER -
  // allocated in SRAM.
  // Load buffer address to register X
  ldi XH, high(UART_RECEIVE_BUFFER)
  ldi XL, low(UART_RECEIVE_BUFFER)

UART_Receive_Str_Loop:            
  // Receive byte and store it in buffer and post-increment buffer address
  // Termination character is included.
  rcall UART_Receive_Byte
  st X+, r16

  // If data is not null-termination byte, continue receiving
  cpi r16, '\0'
  brne UART_Receive_Str_Loop

UART_Receive_Str_Ret: 
  ret

//-----------------------------------------------------------------------------
EEPROM_Read:
  // Load Stack Pointer to register Z
  in ZH, SPH
  in ZL, SPL

  // EEPROM addres that will be read is in SP+3 and SP+4, 
  // high byte and low byte, respectively
  // Load EEPROM address to register X
  ldd XH, Z+3
  ldd XL, Z+4

  // the read data will be stored in buffer allocated in SRAM
  // Load buffer address to register Y
  ldi YH, high(UART_TRANSMIT_BUFFER)
  ldi YL, low(UART_TRANSMIT_BUFFER)

EEPROM_Read_Loop: 
  // Wait for EEPROM to be ready
  sbic EECR, EEPE
  rjmp EEPROM_Read_Loop

  // Set the EEPROM Address Register to address to be read
  out EEARH, XH
  out EEARL, XL
  adiw X, 1 // Post-Increment

  // Read data(byte) from current address by writing EEPROM Read Enable(EERE) -
  // bit in EEPROM Control Register (EECR)
  sbi EECR, EERE

  // Read data from EEPROM Data Register
  // EEDR will be cleared automatically after reading from it
  in  r16, EEDR

  // Store data in buffer and post-increment buffer address
  // Termination character included
  st Y+, r16

  // If data is not null-termination byte, continue reading
  cpi r16, '\0'
  brne EEPROM_Read_Loop

EEPROM_Read_Return:
  // Restore the Stack Pointer and return
  out SPH, ZH
  out SPL, ZL
  ret

//-----------------------------------------------------------------------------
EEPROM_Write:
  // Load Stack Pointer to register Z
  in ZH, SPH
  in ZL, SPL

  // EEPROM addres that will be read is in SP+3 and SP+4, 
  // high byte and low byte, respectively
  // Load EEPROM address to register X
  ldd XH, Z+3
  ldd XL, Z+4

  // the read data will be stored in buffer allocated in SRAM
  // Load buffer address to register Y
  ldi YH, high(UART_RECEIVE_BUFFER)
  ldi YL, low(UART_RECEIVE_BUFFER)

EEPROM_Write_Loop:    
  // Wait for EEPROM to be ready
  sbic EECR, EEPE
  rjmp EEPROM_Write_Loop

  // Set the EEPROM Address Register to address to be written
  out EEARH, XH
  out EEARL, XL
  adiw X, 1 // Post-Increment

  // Load data stored in buffer to r16
  ld r16, Y+

  // Write data to EEDR (EEPROM Data Register)
  out EEDR, r16

  // Set EEMPE(EEPROM Master Write Enable) bit and in EEPROM Control Register
  sbi EECR, EEMPE

  // Enable writing by setting EEPE(Write Enable) bit in EEPROM Control Register
  // Data will be written after this instruction
  sbi EECR, EEPE

  // If data is not null-termination byte, continue writing
  cpi r16, '\0'
  brne EEPROM_Write_Loop

EEPROM_Write_Ret:   
  // Restore Stack Pointer and return
  out SPH, ZH
  out SPL, ZL
  ret

//-----------------------------------------------------------------------------
FN_Toggle:
  // Toggle FN led
  sbi PINC, 3
  rcall DELAY_300ms
  rjmp MAIN

//-----------------------------------------------------------------------------
WPS:
  // AND the r13 with PORTC_MASK, all bits taken out except FN
  ldi r20, PORTC_MASK - 31
  and r13, r20

  // Load the command GET_WLAN_STATUS to r16 and transmit byte
  ldi r16, GET_WLAN_STATUS
  add r16, r13
  rcall UART_Transmit_Byte

  // Toggle FN, if it is on
  sbrc r13, 3
  sbi PINC, 3

  // Wait for any response
  rcall UART_Receive_Byte

  rcall DELAY_300ms

  rjmp MAIN

//-----------------------------------------------------------------------------
PC_CONN:
  // AND the r13 with PORTC_MASK, all bits taken out except FN
  ldi r20, PORTC_MASK - 31
  and r13, r20

  // Load the command GET_PC_STATUS or GET_DESTINATION_IP to r16 and transmit
  ldi r16, GET_PC_STATUS
  add r16, r13 // result is GET_DESTINATION_IP if FN is on
  rcall UART_Transmit_Byte

  // If FN is off, return
  cpi r16, 0x03
  breq PC_CONN_Ret

  // Toggle FN
  sbi PINC, 3

  // Ignore the incoming garbage, wait for response 0xFF
  rcall UART_Receive_Byte
  cpi r16, 0xFF
  brne PC-2

  // Get the new destination IP and store in EEPROM 
  rcall UART_Receive_Str
  eepromWrite HOST_IP_PORT

  // Send destination IP
  rcall IP_SYNC

PC_CONN_Ret:      
  rcall DELAY_300ms
  rjmp MAIN

//-----------------------------------------------------------------------------
IP_SYNC:
  push r16

  // Load the command SET_DESTINATION_IP  to r16 and transmit byte
  ldi r16, SET_DESTINATION_IP
  rcall UART_Transmit_Byte

  // Wait for any response
  rcall UART_Receive_Byte

  // Read the destination IP from EEPROM and send
  eepromRead HOST_IP_PORT
  rcall UART_Transmit_Str

  // Wait until any response
  rcall UART_Receive_Byte

  pop r16
  ret

//-----------------------------------------------------------------------------
MAIN:
  clr r9

  // Load PORTB_MASK to r20 and inverted of it to r21
  ldi r20, PORTB_MASK
  ldi r21, ~PORTB_MASK

  // Pull the current state of PORTB to r10
  in r10, PINB

  // AND the r10 with PORTB_MASK and store result in r10 itself
  // Expected result is 0b00111111 @ PORTB if there is no pressing on switches
  and r10, r20

  // XOR the r10 with the inverted PORTB_MASK and XOR the result with 0
  // Expected result is 0b00000000 if there is no pressing on switches
  eor r21, r10
  eor r21, r8

  // OR the r21 with zero and store result in r9
  // If there is no pressing on switches r9 stays zero
  or r9, r21


  // Load PORTB_MASK to r20 and inverted of it to r21
  ldi r20, PORTC_MASK
  ldi r21, ~PORTC_MASK

  // Pull the current state of PORTC to r11 and copy the result in r13
  in r11, PINC
  mov r13, r11

  // AND the r11 with PORTC_MASK and store result in r11 itself
  // Expected result is 0b00100111 @ PORTC if there is no pressing on switches
  and r11, r20

  // XOR the r11 with the inverted PORTC_MASK and XOR the result with 0
  // Expected result is 0b00000000 if there is no pressing on switches
  eor r21, r11
  eor r21, r8

  // OR the r21 with zero and store result in r9
  // If there is no pressing on switches r9 stays zero
  or r9, r21

  
  // Load PORTD_MASK to r20 and inverted of it to r21
  ldi r20, PORTD_MASK
  ldi r21, ~PORTD_MASK

  // Pull the current state of PORTD to r12
  in r12, PIND

  // AND the r12 with PORTD_MASK and store result in r12 itself
  // Expected result is 0b11101100 @ PORTC if there is no pressing on switches
  and r12, r20

  // XOR the r12 with the inverted PORTC_MASK and XOR the result with 0
  // Expected result is 0b00000000 if there is no pressing on switches
  eor r21, r12
  eor r21, r8

  // OR the r21 with zero and store result in r9
  // If there is no pressing on switches r9 stays zero
  or r9, r21

  // If r9 is zero, there is no pressing on switches, return to loop
  tst r9
  breq MAIN

  // Check if the pressed switch is FN
  sbrs r11, 1
  rjmp FN_Toggle

  // Check if the pressed switch is WPS
  sbrs r10, 4
  rjmp WPS

  // Check if the pressed switch is PCC
  sbrs r12, 5
  rjmp PC_CONN

  // Load the command SEND_KEY to r16 and transmit byte
  ldi r16, SEND_KEY
  rcall UART_Transmit_Byte
  rcall UART_Receive_Byte // Wait for any response

  // AND the r10 with PORTB_MASK, WPS bit taken out
  ldi r20, PORTB_MASK - 16
  and r10, r20

  // AND the r11 with PORTC_MASK, FN bit taken out
  ldi r20, PORTC_MASK - 02
  and r11, r20

  // AND the r12 with PORTD_MASK, PCC bit taken out
  ldi r20, PORTD_MASK - 32
  and r12, r20

  // Toggle FN, if it is on
  sbrc r13, 3
  sbi PINC, 3

  // AND the r13 with PORTC_MASK, all bits taken out except FN
  ldi r20, PORTC_MASK - 31
  and r13, r20
  eor r13, r8

  // Transmit 5 bytes r10, r11, r12, r13, '\0'
  mov r16, r10
  rcall UART_Transmit_Byte
  mov r16, r11
  rcall UART_Transmit_Byte
  mov r16, r12
  rcall UART_Transmit_Byte
  mov r16, r13
  rcall UART_Transmit_Byte
  ldi r16, '\0'
  rcall UART_Transmit_Byte
  rcall UART_Receive_Byte // Wait for any response

  rcall DELAY_300ms
  rjmp MAIN

//-----------------------------------------------------------------------------
DELAY_300ms:
  ldi  r18, 12
  ldi  r19, 57
  ldi  r20, 121
L1: dec  r20
  brne L1
  dec  r19
  brne L1
  dec  r18
  brne L1
  rjmp PC+1
  clr r18
  clr r19
  clr r20
  ret


// Interrupt Vectors
//-----------------------------------------------------------------------------
//INT0: reti
//INT1: reti
//PCINT0: reti
//PCINT1: reti
//PCINT2: reti
//WDT: reti
//TIM2_COMPA: reti
//TIM2_COMPB: reti
//TIM2_OVF: reti
//TIM1_CAPT: reti
//TIM1_COMPA: reti
//TIM1_COMPB: reti
//TIM1_OVF: reti
//TIM0_COMPA: reti
//TIM0_COMPB: reti
//TIM0_OVF: reti
//SPI_STC: reti
//UART_RXC: reti
//UART_UDRE: reti
//UART_TXC: reti
//ADC: reti
//EE_RDY: reti
//ANA_COMP: reti
//TWI: reti
//SPM_RDY: reti

//-----------------------------------------------------------------------------
.eseg

HOST_IP_PORT:   .db "192.168.000.000:00000" , '\0'

//-----------------------------------------------------------------------------
.exit
