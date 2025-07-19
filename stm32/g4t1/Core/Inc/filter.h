/**
 * Martin Egli
 * 2025-07-16
 * mmlib https://github.com/mwuerms/mmlib
 * filter functions
 * + ffir: fir using float
 *
 * see fiiir.com for filter calculations, coeffs n stuff
 */

#ifndef _MM_FILTER_H_
#define _MM_FILTER_H_

// - use c standard libraries --------------------------------------------------
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

// - typedefs ------------------------------------------------------------------

/**
 * FIR filter using float
 */
#define FFIR_SIZE (8)
typedef struct {
    float values[FFIR_SIZE];
    float coeffs[FFIR_SIZE];
    float filtered_value;
    uint16_t size; /// actual size, <=  FFIR_SIZE
} ffir_t;

// - public functions ----------------------------------------------------------

/**
 * initialize a ffir_t with coeffs and to a given size, clear values
 * ffir_t may be reinitialized later
 * @param   ff      pointer to ffir_t
 * @param   coeffs  coefficients to set, nb values: size
 * @param   size    size of ffir, must be <= FFIR_SIZE
 * @return  =true: success, =false: error, invalid pointer or wrong size, > FFIR_SIZE
 */
uint16_t ffir_init(ffir_t *ff, float *coeffs, uint16_t size);

/**
 * clear ffir_t
 * @param   ff      pointer to ffir_t
 * @return  =true: correctly cleared, =false: invalid pointer
 */
uint16_t ffir_clear_values(ffir_t *ff);

/**
 * filter ffir_t
 * @param   ff      pointer to ffir_t
 * @param   value   new value to filter
 * @return =true: new value available in filtered_value, =false: invalid pointer
 */
uint16_t ffir_filter(ffir_t *ff, float value);

#endif // _MM_FILTER_H_
