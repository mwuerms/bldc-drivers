/**
 * Martin Egli
 * 2025-07-16
 * filter module
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#include "stdint.h"
#include "fifo.h"

#define FFILTER_SIZE (4+1)
typedef struct {
	float values[FFILTER_SIZE];
	float mean_value;
	fifo_t fifo;
} ffilter_t;

void ffilter_init(ffilter_t *f);
float ffilter_mean(ffilter_t *f, float value);

#endif // _FILTER_H_
