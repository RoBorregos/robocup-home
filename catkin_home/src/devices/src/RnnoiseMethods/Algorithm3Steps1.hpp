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

#include "DetectionAlgorithms.hpp"

namespace rnnoise_methods {
namespace detection_algorithms {
namespace {
constexpr long MILLIS_PAST_RECORDS = 250L;
constexpr long PAST_RECORD_BUFFER_SIZE = (MILLIS_PAST_RECORDS * SAMPLE_RATE) / 1000L;

// This only includes de actual frames and not past, because we will only
// record at most `ACTUAL_RECORDING_BUFFER_SIZE`.
constexpr long RECORDING_BUFFER_SIZE = ACTUAL_RECORDING_BUFFER_SIZE;

// Constants first part of algorithm.
constexpr int MAX_INIT_MEM = 4;
constexpr int NUM_TO_INIT_VOICE = 4;
constexpr float MIN_PROB_IN_INIT = 0.85;

// Constants second part of algorithm.
constexpr int NUM_ITERATIONS_ALMOST_IN_VOICE = 200L / MS_IN_A_CHUNK;
constexpr float MIN_PROB_IN_ALMOST = 0.90;

// Third part of algorithm.
constexpr int MAX_END_VOICE_MEM = 5;
constexpr int MAX_ITERATIONS_WITHOUT_VOICE = 900L / MS_IN_A_CHUNK;
constexpr int NUM_TO_NOT_END_VOICE = 4;
constexpr float MIN_PROB_IN_END = 0.80;

constexpr int HISTORY_SIZE = MAX_INIT_MEM > MAX_END_VOICE_MEM ? MAX_INIT_MEM : MAX_END_VOICE_MEM;
} // namespace


/**
 * Of type `RnnoiseNewInputProcessor`.
 */
void Algorithm3Steps1ProcessNewInput(const int16_t* input_frames, 
  const RnnoiseProcessor rnnoise_process, const AudioPublisher audio_publisher,
  const bool publish_without_noise) {
  static int16_t recording[RECORDING_BUFFER_SIZE];
  // This index already includes the milliseconds of recording of the past.
  static long next_recording_index = 0;
  static long start_recording_index = 0;

  static bool history[HISTORY_SIZE];
  //ROS_INFO("elementsof(history)= %lu\n", sizeof(history) / sizeof(history[0]));
  static int history_index = 0;

  // Options: 0=looking for voice, 1=confirming voice, 2=in voice.
  static int already_in_voice = 0;

  // Variables first part algorithm.
  static int num_with_voice = 0;

  // Variables second part algorithm.
  static int iterations_almost_in_voice = 0;
  static float sum_prob_almost_in_voice = 0;

  // Variables third part algorithm.
  static int iterations_without_voice = 0;
  static int num_with_voice_in_without_voice = 0;

  
  float prob_voice;
  int16_t frame_values_short[NUMBER_FRAMES_RNNOISE];
  float frame_values_float[NUMBER_FRAMES_RNNOISE];

  // Copy the input frames to a float array.
  convertFromShortArrayToFloatArray(input_frames, NUMBER_FRAMES_RNNOISE, frame_values_float);
  prob_voice = rnnoise_process(frame_values_float, frame_values_float);
  //ROS_INFO("%f", prob_voice);
  //printf("%f\n", prob_voice);

  // Store in the `recording` array in `short` type the new recorded.
  if (publish_without_noise) {
    convertFromFloatArrayToShortArray(frame_values_float, NUMBER_FRAMES_RNNOISE, frame_values_short);
    memcpy(&(recording[next_recording_index]), frame_values_short, NUMBER_FRAMES_RNNOISE * sizeof(int16_t));
  } else {
    memcpy(&(recording[next_recording_index]), input_frames, NUMBER_FRAMES_RNNOISE * sizeof(int16_t));
  }
  next_recording_index = (next_recording_index + NUMBER_FRAMES_RNNOISE) % RECORDING_BUFFER_SIZE;

  if (already_in_voice == 2) {
    ++iterations_without_voice;

    // Remove the last history of yes/no-voice record.
    if (history[history_index] && num_with_voice_in_without_voice > 0) {
      --num_with_voice_in_without_voice;
    }
    
    if (prob_voice > MIN_PROB_IN_END) {
      ++num_with_voice_in_without_voice;

      history[history_index] = true;
      history_index = (history_index + 1) % MAX_END_VOICE_MEM;

      if (num_with_voice_in_without_voice >= NUM_TO_NOT_END_VOICE) {
        // Reset the count because we detect voice.
        iterations_without_voice = 0;

        // NOTE: See if this should be or not.
        // num_with_voice_in_without_voice = 0;
      }
    } else {
      history[history_index] = false;
      history_index = (history_index + 1) % MAX_END_VOICE_MEM;
    }

    if (iterations_without_voice >= MAX_ITERATIONS_WITHOUT_VOICE ||
        next_recording_index == start_recording_index) {
      // The voice never appear again or we reach the limit of frames
      // recorded, then end the recording and save it in a file.

      already_in_voice = 0;
      // With this is enough to the next time and isnt necessary to
      // reset the `history`.
      num_with_voice = 0;

      audio_publisher(
        recording, RECORDING_BUFFER_SIZE, start_recording_index, next_recording_index);

      if (next_recording_index == start_recording_index) {
        // TODO: Maybe knowing this, we can make something to prevent it.
        ROS_INFO("---left(limit of recording)---");
      } else {
        ROS_INFO("---left---");
      }
    }
  } else if (already_in_voice == 1) {
    //printf("%f\n", prob_voice);
    sum_prob_almost_in_voice += prob_voice;

    if (++iterations_almost_in_voice >= NUM_ITERATIONS_ALMOST_IN_VOICE) {
      if (sum_prob_almost_in_voice / iterations_almost_in_voice >= MIN_PROB_IN_ALMOST) {
        already_in_voice = 2;
        iterations_without_voice = 0;

        num_with_voice_in_without_voice = 0;

        ROS_INFO("---entered---");
      } else {
        already_in_voice = 0;

        // With this is enough to the next time and isnt necessary to
        // reset the `history`.
        num_with_voice = 0;

        ROS_INFO("leaving");
      }
    }
  } else {
    if (history[history_index] && num_with_voice > 0) {
      --num_with_voice;
    }
    
    if (prob_voice > MIN_PROB_IN_INIT) {
      ++num_with_voice;

      history[history_index] = true;
      history_index = (history_index + 1) % MAX_INIT_MEM;

      if (num_with_voice >= NUM_TO_INIT_VOICE) {
        already_in_voice = 1;

        start_recording_index = 
          next_recording_index - NUMBER_FRAMES_RNNOISE - PAST_RECORD_BUFFER_SIZE;
        if (start_recording_index < 0) {
          start_recording_index = RECORDING_BUFFER_SIZE + start_recording_index;
        }

        sum_prob_almost_in_voice = 0;
        iterations_almost_in_voice = 0;

        ROS_INFO("entering");
      }
    } else {
      history[history_index] = false;
      history_index = (history_index + 1) % MAX_INIT_MEM;
    }
  }
}

} // namespace detection_algorithms
} // namespace rnnoise_methods
#endif