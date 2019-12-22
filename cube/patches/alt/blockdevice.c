/* 
 * Copyright (c) 2019, Extrems <extrems@extremscorner.org>
 * All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include "../base/common.h"
#include "../base/exi.h"
#include "../base/os.h"

bool exi_probe(int32_t chan)
{
	if (chan == EXI_CHANNEL_2)
		return false;
	if (chan == *(uint8_t *)VAR_EXI_SLOT)
		return false;
	return true;
}

bool exi_trylock(int32_t chan, uint32_t dev, EXIControl *exi)
{
	if (!(exi->state & EXI_STATE_LOCKED) || exi->dev != dev)
		return false;
	if (chan == *(uint8_t *)VAR_EXI_SLOT)
		end_read();
	return true;
}

void di_update_interrupts(void);
void di_complete_transfer(void);

void perform_read(uint32_t offset, uint32_t length, uint32_t address)
{
	*(uint32_t *)VAR_LAST_OFFSET = offset;
	*(uint32_t *)VAR_TMP2 = length;
	*(uint8_t **)VAR_TMP1 = OSPhysicalToUncached(address);
}

void trickle_read(void)
{
	uint32_t position  = *(uint32_t *)VAR_LAST_OFFSET;
	uint32_t remainder = *(uint32_t *)VAR_TMP2;
	uint8_t *data      = *(uint8_t **)VAR_TMP1;
	uint32_t data_size;

	tb_t end;
	mftb(&end);

	if (remainder) {
		data_size = read_frag(data, remainder, position);

		position  += data_size;
		remainder -= data_size;

		*(uint32_t *)VAR_LAST_OFFSET = position;
		*(uint32_t *)VAR_TMP2 = remainder;
		*(uint8_t **)VAR_TMP1 = data + data_size;

		if (!remainder) di_complete_transfer();
	} else if (*(uint32_t *)VAR_DISC_CHANGING) {
		if (tb_diff_usec(&end, (tb_t *)VAR_TIMER_START) > 1000000) {
			*(uint32_t *)VAR_CURRENT_DISC = !*(uint32_t *)VAR_CURRENT_DISC;
			*(uint32_t *)VAR_DISC_CHANGING = false;

			(*DI_EMU)[1] &= ~0b001;
			(*DI_EMU)[1] |=  0b100;
			di_update_interrupts();
		}
	}
}
