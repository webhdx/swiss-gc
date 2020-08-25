/* 
 * Copyright (c) 2019-2020, Extrems <extrems@extremscorner.org>
 * 
 * This file is part of Swiss.
 * 
 * Swiss is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Swiss is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * with Swiss.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../base/common.h"
#include "../base/dolphin/os.h"
#include "../base/emulator.h"

#define SECTOR_SIZE 512

#define sectorBuf OSCachedToUncached(VAR_SECTOR_BUF)

static struct {
	void *buffer;
	uint32_t length;
	uint32_t offset;
	bool read, frag;
} dvd = {0};

static struct {
	uint32_t last_sector;
	int items;
	struct {
		void *address;
		uint32_t length;
		uint32_t offset;
		uint32_t sector;
		read_frag_cb callback;
	} queue[2];
} gcode = {0};

OSAlarm read_alarm = {0};

static void gcode_read_queued(void)
{
	void *address = gcode.queue[0].address;
	uint32_t length = gcode.queue[0].length;
	uint32_t offset = gcode.queue[0].offset;
	uint32_t sector = gcode.queue[0].sector;

	DI[0] = 0b0011000;
	DI[1] = 0;
	DI[2] = 0xB2000000;
	DI[3] = sector;
	DI[4] = SECTOR_SIZE;
	DI[5] = (uint32_t)sectorBuf;
	DI[6] = SECTOR_SIZE;
	DI[7] = 0b011;

	OSUnmaskInterrupts(OS_INTERRUPTMASK_PI_DI);

	gcode.last_sector = sector;
}

static void gcode_done_queued(void)
{
	void *address = gcode.queue[0].address;
	uint32_t length = gcode.queue[0].length;
	uint32_t offset = gcode.queue[0].offset;
	uint32_t sector = gcode.queue[0].sector;
	read_frag_cb callback = gcode.queue[0].callback;

	if (address != VAR_SECTOR_BUF + offset)
		memcpy(address, sectorBuf + offset, length);

	if (--gcode.items) {
		memcpy(gcode.queue, gcode.queue + 1, gcode.items * sizeof(*gcode.queue));
		gcode_read_queued();
	}

	callback(address, length);
}

void do_read_disc(void *address, uint32_t length, uint32_t offset, uint32_t sector, read_frag_cb callback)
{
	int i;

	for (i = 0; i < gcode.items; i++)
		if (gcode.queue[i].callback == callback)
			return;

	sector = offset / SECTOR_SIZE + sector;
	offset = offset % SECTOR_SIZE;
	length = MIN(length, SECTOR_SIZE - offset);

	gcode.queue[i].address = address;
	gcode.queue[i].length = length;
	gcode.queue[i].offset = offset;
	gcode.queue[i].sector = sector;
	gcode.queue[i].callback = callback;
	if (gcode.items++) return;

	if (sector == gcode.last_sector) {
		OSSetAlarm(&read_alarm, READ_COMMAND_LATENCY, (OSAlarmHandler)gcode_done_queued);
		return;
	}

	gcode_read_queued();
}

void di_interrupt_handler(OSInterrupt interrupt, OSContext *context)
{
	OSMaskInterrupts(OS_INTERRUPTMASK_PI_DI);

	gcode_done_queued();
}

void schedule_read(OSTick ticks)
{
	void read_callback(void *address, uint32_t length)
	{
		dvd.buffer += length;
		dvd.length -= length;
		dvd.offset += length;
		dvd.read = !!dvd.length;

		schedule_read(READ_COMMAND_LATENCY);
	}

	OSCancelAlarm(&read_alarm);

	if (!dvd.read) {
		di_complete_transfer();
		return;
	}

	dvd.frag = is_frag_read(dvd.offset, dvd.length);

	if (!dvd.frag)
		read_disc_frag(dvd.buffer, dvd.length, dvd.offset, read_callback);
	else
		OSSetAlarm(&read_alarm, ticks, (OSAlarmHandler)trickle_read);
}

void perform_read(uint32_t address, uint32_t length, uint32_t offset)
{
	dvd.buffer = OSPhysicalToUncached(address);
	dvd.length = length;
	dvd.offset = offset | *VAR_CURRENT_DISC << 31;
	dvd.read = true;

	schedule_read(READ_COMMAND_LATENCY);
}

void trickle_read(void)
{
	if (dvd.read && dvd.frag) {
		OSTick start = OSGetTick();
		int size = read_frag(dvd.buffer, dvd.length, dvd.offset);
		OSTick end = OSGetTick();

		dvd.buffer += size;
		dvd.length -= size;
		dvd.offset += size;
		dvd.read = !!dvd.length;

		schedule_read(OSDiffTick(end, start));
	}
}

void device_reset(void)
{
	end_read();
}

bool change_disc(void)
{
	if (*VAR_SECOND_DISC) {
		*VAR_CURRENT_DISC ^= 1;
		return true;
	}

	return false;
}
