#include <string.h>

#include "periph/cpuid.h"

void cpuid_get(void *id)
{
	memcpy(id, (void *)SYSCON->chip_id, CPUID_LEN);
}

