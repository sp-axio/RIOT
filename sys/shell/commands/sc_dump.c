#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static void dump_1(uint32_t *addr, size_t len, int canonical)
{
	uint8_t *c;
	char *ascii, buffer[78];
	int i, blen;

	memset(buffer, ' ', sizeof(buffer));
	if (canonical) {
		buffer[59] = '|';
		ascii = &buffer[60];
	}
	else {
		buffer[47] = '|';
		ascii = &buffer[48];
	}

	blen = sprintf(buffer, "%08lx  ", (uint32_t)addr);

	if (len > 0)
	{
		for (i=0; i<(len/4); i++) {
			c = (uint8_t *)&addr[i];
			if (canonical) {
				blen += sprintf(buffer + blen, "%02x %02x %02x %02x ", c[0], c[1], c[2], c[3]);
				*ascii++ = isprint(c[0])?c[0]:'.';
				*ascii++ = isprint(c[1])?c[1]:'.';
				*ascii++ = isprint(c[2])?c[2]:'.';
				*ascii++ = isprint(c[3])?c[3]:'.';
			}
			else {
				blen += sprintf(buffer + blen, "%08lx ", addr[i]);
				*ascii++ = isprint(c[3])?c[3]:'.';
				*ascii++ = isprint(c[2])?c[2]:'.';
				*ascii++ = isprint(c[1])?c[1]:'.';
				*ascii++ = isprint(c[0])?c[0]:'.';
			}
		}

		c = (uint8_t *)&addr[i];
		if (canonical) {
			for (i=0; i<(len%4); i++) {
				blen += sprintf(buffer + blen, "%02x ", c[i]);
				*ascii++ = isprint(c[i])?c[i]:'.';
			}
			buffer[blen] = ' ';
		}
		else if (len%4) {
			buffer[blen] = ' ';
			blen += (2*(4-(len%4)));
			ascii += (4-(len%4));
			for (i=(len%4)-1; i>=0; i--) {
				blen += sprintf(buffer + blen, "%02x", c[i]);
				*ascii++ = isprint(c[i])?c[i]:'.';
			}
			//buffer[blen] = ' '; /* don't display ascii value */
		}

		*ascii++ = '|';
		*ascii++ = '\0';
	}
	puts(buffer);
}

void hexdump(uint32_t *addr, int len, int canonical)
{
	while (len>=16) {
		dump_1(addr, 16, canonical);
		addr += 4;
		len -= 16;
	}

	if (len < 0)
		len = 0;

	dump_1(addr, len, canonical);
}

int _cmd_dump(int argc, char **argv)
{
	uint32_t *addr;
	int len, canonical;

	if (argc == 3) {
		addr = (uint32_t *)strtoul(argv[1], NULL, 16);
		len = (int)strtoul(argv[2], NULL, 16);
		canonical = 0;
	}
	else if (argc == 4) {
		addr = (uint32_t *)strtoul(argv[1], NULL, 16);
		len = (int)strtoul(argv[2], NULL, 16);
		canonical = strtoul(argv[3], NULL, 16) ? 1 : 0;
	}
	else {
		printf("Usage: %s address(hex) len(hex) {canonical display:1,0}\n", argv[0]);
		return 1;
	}

	hexdump(addr, len, canonical);

	return 0;
}

