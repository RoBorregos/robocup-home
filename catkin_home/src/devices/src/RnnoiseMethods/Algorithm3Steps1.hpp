#ifndef RNNOISE_METHODS__ALGORITHM_3_STEPS_1_HPP
#define RNNOISE_METHODS__ALGORITHM_3_STEPS_1_HPP

#include "DetectionAlgorithms.hpp"

namespace rnnoise_methods {
namespace detection_algorithms {
namespace {
constexpr int NUMBER_CHUNKS_PAST_RECORDS = 13L;
constexpr long PAST_RECORD_BUFFER_SIZE = NUMBER_CHUNKS_PAST_RECORDS * NUMBER_FRAMES_RNNOISE;

constexpr long RECORDING_BUFFER_SIZE = ACTUAL_RECORDING_BUFFER_SIZE + PAST_RECORD_BUFFER_SIZE;

// Constants first part of algorithm.
constexpr int MAX_INIT_MEM = 4;
constexpr int NUM_TO_INIT_VOICE = 4;
constexpr float MIN_PROB_IN_INIT = 0.85;

// Constants second part of algorithm.
constexpr int NUM_ITERATIONS_ALMOST_IN_VOICE = 300L / MS_IN_A_CHUNK;
constexpr float MIN_PROB_IN_ALMOST = 0.85;

// Third part of algorithm.
constexpr int MAX_END_VOICE_MEM = 5;
constexpr int MAX_ITERATIONS_WITHOUT_VOICE = 900L / MS_IN_A_CHUNK;
constexpr int NUM_TO_END_VOICE = 4;
constexpr float MIN_PROB_IN_END = 0.80;

constexpr int HISTORY_SIZE = MAX_INIT_MEM > MAX_END_VOICE_MEM ? MAX_INIT_MEM : MAX_END_VOICE_MEM;
} // namespace


/**
 * @param {const int16_t*} input_frames An array with `NUMBER_FRAMES_RNNOISE` elements.
 */
void Algorithm3Steps1ProcessNewInput(const int16_t* input_frames, 
  const RnnoiseProcessor rnnoise_process, const AudioPublisher audio_publisher) {
  static int16_t recording[RECORDING_BUFFER_SIZE];
  // This index already includes the milliseconds of recording of the past.
  static long recording_index = 0;
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


  if (already_in_voice == 2) {
    prob_voice = rnnoise_process(frame_values_float, frame_values_float);

    // Store in the `recording` array in `short` type the new recorded.
    convertFromFloatArrayToShortArray(frame_values_float, NUMBER_FRAMES_RNNOISE, frame_values_short);
    memcpy(&(recording[recording_index]), frame_values_short, NUMBER_FRAMES_RNNOISE * sizeof(int16_t));
    recording_index = (recording_index + NUMBER_FRAMES_RNNOISE) % RECORDING_BUFFER_SIZE;


    ++iterations_without_voice;

    // Remove the last history of yes/no-voice record.
    if (history[history_index] && num_with_voice_in_without_voice > 0) {
      --num_with_voice_in_without_voice;
    }
    
    if (prob_voice > MIN_PROB_IN_END) {
      ++num_with_voice_in_without_voice;

      history[history_index] = true;
      history_index = (history_index + 1) % MAX_END_VOICE_MEM;

      if (num_with_voice_in_without_voice >= NUM_TO_END_VOICE) {
        // Reset the count because we detect voice.
        iterations_without_voice = 0;

        // NOTE: See if this should be or not.
        // num_with_voice_in_without_voice = 0;
      }
    } else {
      history[history_index] = false;
      history_index = (history_index + 1) % MAX_END_VOICE_MEM;
    }


    // TODO: Ensure that this check since `ACTUAL_RECORDING_BUFFER_SIZE`.
    if (iterations_without_voice >= MAX_ITERATIONS_WITHOUT_VOICE ||
        recording_index == start_recording_index) {
      // The voice never appear again or we reach the limit of frames
      // recorded, then end the recording and save it in a file.

      already_in_voice = 0;
      // With this is enough to the next time and isnt necessary to
      // reset the `history`.
      num_with_voice = 0;

      audio_publisher(
        recording, RECORDING_BUFFER_SIZE, start_recording_index, recording_index);

      if (recording_index == start_recording_index) {
        // TODO: Maybe knowing this, we can make something to prevent it.
        ROS_INFO("---left(limit of recording)---");
      } else {
        ROS_INFO("---left---");
      }
    }

  } else if (already_in_voice == 1) {
    prob_voice = rnnoise_process(frame_values_float, frame_values_float);

    // Store in the `recording` array in `short` type the new recorded.
    convertFromFloatArrayToShortArray(frame_values_float, NUMBER_FRAMES_RNNOISE, frame_values_short);
    memcpy(&(recording[recording_index]), frame_values_short, NUMBER_FRAMES_RNNOISE * sizeof(int16_t));
    recording_index = (recording_index + NUMBER_FRAMES_RNNOISE) % RECORDING_BUFFER_SIZE;


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
    // NOTE: Here we can use this to speed up, but we lost the memory for
    // the other parts of the nn.
    // prob_voice = rnnoise_process_frame_only_voice_prob(st, x, x);
    prob_voice = rnnoise_process(frame_values_float, frame_values_float);

    if (history[history_index] && num_with_voice > 0) {
      --num_with_voice;
    }
    
    if (prob_voice > MIN_PROB_IN_INIT) {
      ++num_with_voice;

      history[history_index] = true;
      history_index = (history_index + 1) % MAX_INIT_MEM;

      if (num_with_voice >= NUM_TO_INIT_VOICE) {
        already_in_voice = 1;

        start_recording_index = recording_index - PAST_RECORD_BUFFER_SIZE;
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

    // Save this to the recording array.
    convertFromFloatArrayToShortArray(frame_values_float, NUMBER_FRAMES_RNNOISE, frame_values_short);
    memcpy(&(recording[recording_index]), frame_values_short, NUMBER_FRAMES_RNNOISE * sizeof(int16_t));
    recording_index = (recording_index + NUMBER_FRAMES_RNNOISE) % RECORDING_BUFFER_SIZE;
  }
}

} // namespace detection_algorithms
} // namespace rnnoise_methods
#endif