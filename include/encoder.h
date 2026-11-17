#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize both quadrature encoders. Counts are signed and use x4 decoding.
void encoder_init(void);

// Read an atomic snapshot of the cumulative left and right counts.
void encoder_get_counts(int32_t* left, int32_t* right);

// Set both counts to zero and resynchronize the decoder to the current levels.
void encoder_reset_counts(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
