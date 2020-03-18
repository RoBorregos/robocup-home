#include "RnnoiseMethods/AlgorithmAccTolerance1.hpp"

#include <cstdio>

#include <algorithm>

#include "ros/ros.h"

#include "RnnoiseMethods/HelperMethods.hpp"

namespace rnnoise_methods {
namespace detection_algorithms {
namespace {
constexpr long RECORDING_BUFFER_SIZE = ACTUAL_RECORDING_BUFFER_SIZE;

constexpr float MIN_PROB_VOICE_LOW = 0.55F;
constexpr float MIN_PROB_VOICE_HIGH = 0.80F;

constexpr long INCREMENT_TOLERANCE_LOW = 1L;
constexpr long INCREMENT_TOLERANCE_HIGH = 2L;
constexpr long DECREMENT_TOLERANCE = 1L;
constexpr long MAX_TOLERANCE_MILLIS = 700L;
constexpr long MAX_TOLERANCE_ITERATIONS = MAX_TOLERANCE_MILLIS / MS_IN_A_CHUNK;
constexpr long MIN_TOLERANCE_ITERATIONS = 0L;

constexpr long MIN_RECORDING_LENGTH_MILLIS = 600L;
constexpr long MIN_RECORDING_LENGTH_ITERATIONS = MIN_RECORDING_LENGTH_MILLIS / MS_IN_A_CHUNK;
} // namespace


void AlgorithmAccTolerance1ProcessNewInput(const int16_t* input_frames, 
  const RnnoiseProcessor rnnoise_process, const AudioPublisher audio_publisher,
  const bool publish_without_noise) {
  static int16_t recording[RECORDING_BUFFER_SIZE];
  static long next_recording_index = 0;
  static long start_recording_index = 0;

  static long tolerance_iterations_left = MIN_TOLERANCE_ITERATIONS;
  static long recording_length = 0;


  int16_t frame_values_short[NUMBER_FRAMES_RNNOISE];
  float frame_values_float[NUMBER_FRAMES_RNNOISE];

  // Copy the input frames to a float array.
  convertFromShortArrayToFloatArray(input_frames, NUMBER_FRAMES_RNNOISE, frame_values_float);
  const float prob_voice = rnnoise_process(frame_values_float, frame_values_float);

  // Store in the `recording` array in `short` type the new recorded.
  if (publish_without_noise) {
    convertFromFloatArrayToShortArray(frame_values_float, NUMBER_FRAMES_RNNOISE, frame_values_short);
    memcpy(&(recording[next_recording_index]), frame_values_short, NUMBER_FRAMES_RNNOISE * sizeof(int16_t));
  } else {
    memcpy(&(recording[next_recording_index]), input_frames, NUMBER_FRAMES_RNNOISE * sizeof(int16_t));
  }
  next_recording_index = (next_recording_index + NUMBER_FRAMES_RNNOISE) % RECORDING_BUFFER_SIZE;

  ++recording_length;
  if (prob_voice >= MIN_PROB_VOICE_LOW) {
    // Check if the voice is just starting.
    if (tolerance_iterations_left == MIN_TOLERANCE_ITERATIONS) {
      ROS_INFO("entering");
      start_recording_index = next_recording_index - NUMBER_FRAMES_RNNOISE;
      if (start_recording_index < 0) {
        start_recording_index = RECORDING_BUFFER_SIZE + start_recording_index;
      }

      recording_length = 1; //
    }

    // Check how many tolerance the probability of voice will sum.
    if (prob_voice >= MIN_PROB_VOICE_HIGH) {
      tolerance_iterations_left = std::min(
        tolerance_iterations_left + INCREMENT_TOLERANCE_HIGH, MAX_TOLERANCE_ITERATIONS);
    } else {
      tolerance_iterations_left = std::min(
        tolerance_iterations_left + INCREMENT_TOLERANCE_LOW, MAX_TOLERANCE_ITERATIONS);
    }
  } else {
    tolerance_iterations_left = 
      std::max(tolerance_iterations_left - DECREMENT_TOLERANCE, MIN_TOLERANCE_ITERATIONS);

    // Check if the (tolerance for the) voice just finished.
    if (tolerance_iterations_left == MIN_TOLERANCE_ITERATIONS) {
      if (recording_length >= MIN_RECORDING_LENGTH_ITERATIONS) {
        // The recording is long enough, let's publish it.
        audio_publisher(
          recording, RECORDING_BUFFER_SIZE, start_recording_index, next_recording_index);
        ROS_INFO("---left---");
      }

      recording_length = 0;
    }
  }

  // If the recording is in the limit of length, let's publish it and restart the state.
  if (recording_length > 0 && start_recording_index == next_recording_index) {
    audio_publisher(
      recording, RECORDING_BUFFER_SIZE, start_recording_index, next_recording_index);
    ROS_INFO("---left (limit of recording)---");

    recording_length = 0;
    tolerance_iterations_left = MIN_TOLERANCE_ITERATIONS;
  }

  //printf("%ld  [%f]\n", tolerance_iterations_left, prob_voice);
}

} // namespace detection_algorithms
} // namespace rnnoise_methods
