/***************************************************************************
* Wiikey Fusion related patches
* emu_kidid 2015
***************************************************************************/

#include "../../reservedarea.h"

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

extern char _readsector[];
extern int _readsectorsize;

#define READ_SECTOR ((char*)&_readsector)
#define READ_SECTOR_SIZE ((int)_readsectorsize)
extern int usb_sendbuffer_safe(const void *buffer,int size);
extern void print_int_hex(unsigned int num);

void wkfWriteOffset(u32 offset) {
	static volatile u32* const wkf = (u32*)0xCC006000;
	wkf[2] = 0xDE000000;
	wkf[3] = offset;
	wkf[4] = 0x5A000000;
	wkf[6] = 0;
	wkf[8] = 0;
	wkf[7] = 1;
	while( wkf[7] & 1);
}

// Length is always 0x800
void __wkfReadSector(void* dst, u32 offset) {
	static volatile unsigned int* const wkf = (unsigned int*)0xCC006000;

	wkf[2] = 0xA8000000;
	wkf[3] = offset >> 2;
	wkf[4] = READ_SECTOR_SIZE;
	wkf[5] = (u32)dst&0x01FFFFFF;
	wkf[6] = READ_SECTOR_SIZE;
	wkf[7] = 3; // DMA | START
	dcache_flush_icache_inv(dst, READ_SECTOR_SIZE);
	while(wkf[7] & 1);
}

void* mymemcpy(void* dest, const void* src, u32 count)
{
	char* tmp = (char*)dest,* s = (char*)src;

	while (count--)
		*tmp++ = *s++;

	return dest;
}

void wkfRead(void* dst, int len, u32 offset)
{
	/*asm("mfmsr	5");
	asm("lis 3, 0x8000");
	asm("stw 5, 0x9FC(3)");
	asm("rlwinm	5,5,0,17,15");
	asm("mtmsr	5");*/
	u8 *sector_buffer = (u8*)READ_SECTOR;
	while (len)
	{
		__wkfReadSector(sector_buffer, (offset-(offset%READ_SECTOR_SIZE)));
		u32 off = offset & (READ_SECTOR_SIZE-1);

		int rl = READ_SECTOR_SIZE - off;
		if (rl > len)
			rl = len;
		mymemcpy(dst, sector_buffer + off, rl);	

		offset += rl;
		len -= rl;
		dst += rl;
	}
	/*asm("lis 3, 0x8000");
	asm("lwz 5, 0x9FC(3)");
	asm("mtmsr	5");*/
}

// Adjusts the offset on the WKF for fragmented reads
void adjust_read() {
	volatile unsigned long* dvd = (volatile unsigned long*)0xCC006000;
	
	u32 dst = dvd[5];
	u32 len = dvd[4];
	u32 offset = (dvd[3]<<2);
	
	u32 *fragList = (u32*)VAR_FRAG_LIST;
	int isDisc2 = (*(u32*)(VAR_DISC_2_LBA)) == (*(u32*)VAR_CUR_DISC_LBA);
	int maxFrags = (*(u32*)(VAR_DISC_2_LBA)) ? ((VAR_FRAG_SIZE/12)/2) : (VAR_FRAG_SIZE/12), i = 0, j = 0;
	int fragTableStart = isDisc2 ? (maxFrags*4) : 0;
	u32 adjustedOffset = offset;

	// Locate this offset in the fat table
	for(i = 0; i < maxFrags; i++) {
		int fragOffset = fragList[(i*3)+0];
		int fragSize = fragList[(i*3)+1];
		int fragSector = fragList[(i*3)+2];
		int fragOffsetEnd = fragOffset + fragSize;
		
		// Find where our read starts
		if(offset >= fragOffset && offset <= fragOffsetEnd) {
			if(fragOffset != 0) {
				adjustedOffset = offset - fragOffset;
			}
			fragSector = fragSector + (adjustedOffset>>9);
			if(*(volatile unsigned int*)VAR_TMP1 != fragSector) {
				wkfWriteOffset(fragSector);
				*(volatile unsigned int*)VAR_TMP1 = fragSector;
			}
			wkfRead((void*)dst, len, adjustedOffset & 511);
			break;
		}
	}
}

void swap_disc() {
	int isDisc1 = (*(u32*)(VAR_DISC_1_LBA)) == (*(u32*)VAR_CUR_DISC_LBA);
	*(u32*)VAR_CUR_DISC_LBA = isDisc1 ? *(u32*)(VAR_DISC_2_LBA) : *(u32*)(VAR_DISC_1_LBA);
}

void fake_lid_interrupt() {

}
