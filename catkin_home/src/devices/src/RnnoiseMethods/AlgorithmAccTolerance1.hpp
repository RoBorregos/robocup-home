/**
 * This file contains an algorithm that accumulates the times with
 * voice to amortage the time we wait for more voice.
 *
 * The idea is that we have an accumulator that will say how much
 * time this will wait for more voice to come. It increases each
 * time there is voice and decreases when not. 
 *
 * TODO: Maybe with an array use several sections to increment
 * differently the tolerance instead of only 2. Also, decimal
 * values could help.
 */
#ifndef RNNOISE_METHODS__ALGORITHM_ACC_TOLERANCE_1_HPP
#define RNNOISE_METHODS__ALGORITHM_ACC_TOLERANCE_1_HPP

#include <cstdint>

#include "RnnoiseMethods/DetectionAlgorithms.hpp"

namespace rnnoise_methods {
namespace detection_algorithms {
/**
 * Of type `RnnoiseNewInputProcessor`.
 */
void AlgorithmAccTolerance1ProcessNewInput(const int16_t* input_frames, 
  const RnnoiseProcessor rnnoise_process, const AudioPublisher audio_publisher,
  const bool publish_without_noise);

} // namespace detection_algorithms
} // namespace rnnoise_methods
#endif