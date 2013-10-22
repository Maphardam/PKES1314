#define __SFR_OFFSET 0
#include "avr/io.h"

#define off   r0
#define temp  r16
#define time1 r17
#define time2 r18
#define time3 r19

.global main

main: 
  ldi   temp, 0xFF
  out   DDRB, temp

loop:
  rcall pause
  rcall led_switch
  rjmp  loop
  
led_switch:
  sbi   PINB, 7
  ret

pause:
  ldi   time1, 0xFF
pause_run:
  dec   time1
  rcall pause2
  cpse  time1, off
  rcall pause_run
  ret
  
pause2:
  ldi   time2, 255
pause2_run:
  dec   time2
  rcall pause3
  cpse  time2, off
  rjmp  pause2_run
  ret
  
pause3:
  ldi   time3, 255
pause3_run:
  dec   time3
  cpse  time3, off
  rjmp  pause3_run
  ret
