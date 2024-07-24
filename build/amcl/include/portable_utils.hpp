#ifndef PORTABLE_UTILS_H
#define PORTABLE_UTILS_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif
#define HAVE_DRAND48
#ifndef HAVE_DRAND48
// Some system (e.g., Windows) doesn't come with drand48(), srand48().
// Use rand, and srand for such system.
static double drand48(void)
{
	long long d = rand() - 1;
	if (d < 0) d = 0;
    return ((double)(d))/RAND_MAX;
}

static void srand48(long int seedval)
{
    srand(seedval);
}
#endif

#ifdef __cplusplus
}
#endif

#endif