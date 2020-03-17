/**
 * This files contains a 3 steps algorithm and all its information for a
 * Detector Algorithm of the type `RnnoiseNewInputProcessor`.
 *
 * In general it has 3 stages:
 * 1.- Looks for `NUM_TO_INIT_VOICE` chunks that go more than
 *   `MIN_PROB_IN_INIT` in a group of `MAX_INIT_MEM` adjacent elements.
 * 2.- Looks for an average of at least `MIN_PROB_IN_ALMOST` in 
 *   `NUM_ITERATIONS_ALMOST_IN_VOICE` iterations. After this, we mark
 *   that we are in voice.
 * 3.- To say that the voice ended: this looks for `MAX_ITERATIONS_WITHOUT_VOICE`
 *   iterations without at least `MIN_PROB_IN_END` probability in 
 *   `NUM_TO_NOT_END_VOICE` chunks in `MAX_END_VOICE_MEM` adjacent ones.
 *
 * This one takes for the recording, when entering to stage 1, few
 * records from before that point.
 */

#ifndef RNNOISE_METHODS__ALGORITHM_3_STEPS_1_HPP
#define RNNOISE_METHODS__ALGORITHM_3_STEPS_1_HPP

#include <cstdint>

#include "RnnoiseMethods/DetectionAlgorithms.hpp"

namespace rnnoise_methods {
namespace detection_algorithms {
/**
 * Of type `RnnoiseNewInputProcessor`.
 */
void Algorithm3Steps1ProcessNewInput(const int16_t* input_frames, 
  const RnnoiseProcessor rnnoise_process, const AudioPublisher audio_publisher,
  const bool publish_without_noise);

} // namespace detection_algorithms
} // namespace rnnoise_methods
#endif