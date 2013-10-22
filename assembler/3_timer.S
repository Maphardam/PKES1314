#define __SFR_OFFSET 0
#include "avr/io.h"

#define temp	r16

.global main
main: 
  ldi   temp, 0xFF  
  out   DDRB, temp           
  ldi   temp, 0b00000101     
  out   TCCR0B, temp           ; set prescaler to 1024
  
loop:
  rcall led_switch    
  rcall pause                  ; WAIT
  rcall pause                  ; WAIT
  rcall pause                  ; WAIT
  rcall pause                  ; WAIT
  rjmp  loop
  
led_switch:
  sbi   PINB, 7                ; FLIP THE 7 BIT
  ret

pause:          
  in    temp, TIFR0           ; wait for timer overrun
  andi  temp, 0b00000010      ; (TOV0)
  breq  pause                 ; branch if equal
  ldi   temp, 0b00000010      ; reset flag
  out   TIFR0, temp           ; ...
  ret