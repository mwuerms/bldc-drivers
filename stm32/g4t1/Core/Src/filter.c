/**
 * Martin Egli
 * 2025-07-16
 * filter module
 */
#include "filter.h"
#include "stddef.h"

void ffilter_init(ffilter_t *f) {
	if(f == NULL) {
		// error, invalid filter
		return;
	}
	fifo_init(&(f->fifo), f->values, FFILTER_SIZE);
	f->mean_value = 0.0f;
}

float ffilter_mean(ffilter_t *f, float value) {
	float sum;
	if(f == NULL) {
		// error, invalid filter
	}
	if(fifo_try_append(&(f->fifo)) == true) {
		f->values[f->fifo.wr_proc] = values;
		if(fifo_finalize_append(&(f->fifo)) == true) {
			// data in fifo
		}
	}

}
