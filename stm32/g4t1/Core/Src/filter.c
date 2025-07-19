/**
 * Martin Egli
 * 2025-07-16
 * mmlib https://github.com/mwuerms/mmlib
 * filter functions
 * + ffir: fir using float
 */

#include "filter.h"

// - public functions ----------------------------------------------------------

uint16_t ffir_init(ffir_t *ff, float *coeffs, uint16_t size) {
    uint16_t n;
    if(ff == NULL) {
        // error, invalid filter
        return false;
    }
    if((size > FFIR_SIZE) || (size == 0)) {
        // error, invalid size
        return false;
    }
    for(n = 0; n < size; n++) {
        ff->coeffs[n] = coeffs[n];
    }
    ff->size = size;
    return ffir_clear_values(ff);
}

uint16_t ffir_clear_values(ffir_t *ff)  {
    uint16_t n;
    if(ff == NULL) {
        // error, invalid filter
        return false;
    }
    ff->filtered_value = 0;
    for(n = 0; n < ff->size; n++) {
        ff->values[n] = 0.0f;
    }
    return true;
}

uint16_t ffir_filter(ffir_t *ff, float value) {
    uint16_t n;
    if(ff == NULL) {
        // error, invalid filter
        return false;
    }
    ff->filtered_value = 0;
    for(n = (ff->size-1); n != 0; n--) {
        ff->values[n] = ff->values[n-1]; // move previous value here
        ff->filtered_value += ff->values[n] * ff->coeffs[n]; // calc fir
    }
    ff->values[0] = value;
    ff->filtered_value += ff->values[0] * ff->coeffs[0];
    return true;
}
