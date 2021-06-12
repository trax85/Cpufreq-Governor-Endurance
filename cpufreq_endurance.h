#ifndef _CPUFREQ_PERFPLUS_H_
#define _CPUFREQ_PERFPLUS_H_

#include <linux/string.h>
#include <linux/ctype.h>
/*
 *  ======== atoi ========
 *  Purpose:
 *      This function converts strings in decimal or hex format to integers.
 */
static s32 atoi(char *psz_buf)
{
	char *pch = psz_buf;
	s32 base = 0;

	while (isspace(*pch))
		pch++;

	if (*pch == '-' || *pch == '+') {
		base = 10;
		pch++;
	} else if (*pch && tolower(pch[strlen(pch) - 1]) == 'h') {
		base = 16;
	}

	return simple_strtoul(pch, NULL, base);
}

#endif
