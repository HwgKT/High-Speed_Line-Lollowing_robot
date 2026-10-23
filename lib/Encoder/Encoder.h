#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

namespace Encoder {

void begin();
void counts(int32_t& left, int32_t& right);
void reset();

}  // namespace Encoder

#endif