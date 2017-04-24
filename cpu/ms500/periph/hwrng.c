#include <string.h>

#include "cpu.h"

void hwrng_init(void)
{
    tTRNG_REG* baseAddr = TRNG;
    uint32_t   runVal = 0;

    baseAddr->smode = 0x000A0100;
    baseAddr->ctrl  = 0x2UL;

    do {
        runVal = baseAddr->stat;
        runVal &= 0x0200;
    } while ( runVal != 0x200 );
}

void hwrng_read(uint8_t *buf, unsigned int num)
{
	tTRNG_REG* baseAddr = TRNG;

	int remaining = num;
	int copyLen;

	memset(buf, 0, num);

	do {
		baseAddr->int_stat = 1; //TRNG_IRQ_RAND_RDY_MASK;
		baseAddr->mode     = 1UL << 3; //TRNG_MODE_R256;
		baseAddr->ctrl     = 1; //TRNG_CMD_GEN_RAND;
		while( ((baseAddr->int_stat & 1 /*TRNG_IRQ_RAND_RDY_MASK*/) == 0) ||
				((baseAddr->stat & 30 /*TRNG_STAT_GENERATING*/) == 30 /*TRNG_STAT_GENERATING*/) );

		copyLen = remaining;
		if (copyLen > 32)
			copyLen = 32;
		memcpy(buf + num - remaining, (void*)&baseAddr->rand_base, copyLen);
		remaining -= copyLen;
	} while(remaining > 0);
}

