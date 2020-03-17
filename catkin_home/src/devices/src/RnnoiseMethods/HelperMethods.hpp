#ifndef RNNOISE_METHODS__HELPER_METHODS_HPP
#define RNNOISE_METHODS__HELPER_METHODS_HPP

#include <cstdint>

namespace rnnoise_methods {
void convertFromShortArrayToFloatArray(
  const int16_t *in_short_array, const long num_elements, float *out_float_array);

void convertFromFloatArrayToShortArray(
  const float *in_float_array, const long num_elements, int16_t *out_short_array);
} // namespace rnnoise_methods

#endif
