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
				lds r25, counter_trigger   ;load current trigger value
				lds r26, counter_value     ;
				; Einlesen der Buttons
				sbis _SFR_IO_ADDR(PING), PING5 ; set if button 1
				ldi  r24, 1
				; vervollständige		   ; set if button 2

				; Aktualisieren des Triggers
		
				; Aktualisieren des Counters
		
; decrement_counter

; increment_counter

end:			sts counter_dir, r24
				sts counter_trigger, r25   ;save counter_trigger
				sts counter_value, r26     ;
