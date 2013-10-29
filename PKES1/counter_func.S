/*
 * counter_func.S
 *
 *  Created on: Sep 20, 2013
 *      Author: zug
 */

#include <avr/io.h>
#include "def.h"

.global counter_func

.extern counter_trigger
.extern counter_dir
.extern counter_value

counter_func:	lds r24, counter_dir
				lds r25, counter_trigger      ; load current trigger value
				lds r26, counter_value

				sbic _SFR_IO_ADDR(PINF), PINF4 ; set if button 1 (up)
				ldi  r24, 1
				sbic _SFR_IO_ADDR(PING), PING5 ; set if button 2 (down)
				ldi  r24, 0

				; Aktualisieren des Triggers
				dec  r25

				;cpi  r25, 0
				tst  r25
				breq update_counter
				jmp  end

update_counter: ; Aktualisieren des Counters
				cpi  r24, 1
				breq increment
				dec  r26 ; decrement
				jmp  end

increment:		inc  r26

end:			sts counter_dir, r24
				sts counter_trigger, r25   ;save counter_trigger
				sts counter_value, r26     ;
