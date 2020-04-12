#include "RnnoiseMethods/HelperMethods.hpp"

namespace rnnoise_methods {
void convertFromShortArrayToFloatArray(
  const int16_t *in_short_array, const long num_elements, float *out_float_array) {
  for (long i = 0; i < num_elements; ++i) {
    out_float_array[i] = in_short_array[i];
  }
}

void convertFromFloatArrayToShortArray(
  const float *in_float_array, const long num_elements, int16_t *out_short_array) {
  for (long i = 0; i < num_elements; ++i) {
    out_short_array[i] = in_float_array[i];
  }
}
} // namespace rnnoise_methods