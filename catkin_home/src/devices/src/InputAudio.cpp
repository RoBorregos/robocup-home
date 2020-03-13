/**
 * TODO: Fix names to include `int16_t` instead of `short`.
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "ros/package.h"
#include "audio_common_msgs/AudioData.h"
extern "C" {
#include "rnnoise.h"
}

#include "RnnoiseMethods/HelperMethods.hpp"


using namespace std;


constexpr int ADVERTISE_BUFFER_SIZE = 10;

constexpr int NUMBER_FRAMES_RNNOISE = 480;
constexpr long SAMPLE_RATE = 48000L;
constexpr int MS_IN_A_CHUNK = (NUMBER_FRAMES_RNNOISE * 1000L) / SAMPLE_RATE;

constexpr int NUMBER_CHUNKS_PAST_RECORDS = 13L;
constexpr long PAST_RECORD_BUFFER_SIZE = NUMBER_CHUNKS_PAST_RECORDS * NUMBER_FRAMES_RNNOISE;

constexpr int SECONDS_ACTUAL_RECORDING = 10;
constexpr long ACTUAL_RECORDING_BUFFER_SIZE = SECONDS_ACTUAL_RECORDING * SAMPLE_RATE;

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


ros::Publisher publi;

DenoiseState *st;


void RNNoiseWarmUp(DenoiseState *st) {
  int i;
  float x[NUMBER_FRAMES_RNNOISE];

  FILE *f1;
  const string pkg_path = ros::package::getPath("devices");
  f1 = fopen((pkg_path + "/data/rnnoise_warmup_street_20dB.raw").c_str(), "r");

  while (true) {
    int16_t tmp[NUMBER_FRAMES_RNNOISE];
    fread(tmp, sizeof(int16_t), NUMBER_FRAMES_RNNOISE, f1);
    if (feof(f1)) break;

    convertFromShortArrayToFloatArray(tmp, NUMBER_FRAMES_RNNOISE, x);

    rnnoise_process_frame(st, x, x);
  }
  
  fclose(f1);
}

/*
 * The `begin_index_element` is inclusive and the `end_index_element` is exclusive.
 * `recording` is not modified.
 */
void PublishAudioWithoutNoise(int16_t recording[RECORDING_BUFFER_SIZE],
  const long begin_index_element, const long end_index_element) {
  vector<uint8_t> output_vector;

  if (begin_index_element < end_index_element ) {
    // The recording doesn't go circular.
    output_vector.assign(
      (uint8_t*)(recording + begin_index_element),
      (uint8_t*)(recording + end_index_element));
  } else {
    // The recoding goes circular and we have to save it in two steps.
    const long length_int8_elements = 
      (RECORDING_BUFFER_SIZE - begin_index_element + end_index_element) 
      * sizeof(int16_t);
    output_vector.reserve(length_int8_elements);

    output_vector.insert(output_vector.end(),
      (uint8_t*)(recording + begin_index_element),
      (uint8_t*)(recording + RECORDING_BUFFER_SIZE));
    output_vector.insert(output_vector.end(), 
      (uint8_t*)recording,
      (uint8_t*)(recording + end_index_element));
  }

  audio_common_msgs::AudioData msg_audio;
  msg_audio.data = output_vector;

  publi.publish(msg_audio);
}

/**
 * @param {const int16_t*} input_frames An array with `NUMBER_FRAMES_RNNOISE` elements.
 */
void RNNoiseProcessNewInput(const int16_t* input_frames) {
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
      prob_voice = rnnoise_process_frame(st, frame_values_float, frame_values_float);

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


      if (iterations_without_voice >= MAX_ITERATIONS_WITHOUT_VOICE ||
          recording_index == start_recording_index) {
        // The voice never appear again or we reach the limit of frames
        // recorded, then end the recording and save it in a file.

        already_in_voice = 0;
        // With this is enough to the next time and isnt necessary to
        // reset the `history`.
        num_with_voice = 0;

        PublishAudioWithoutNoise(recording, start_recording_index, recording_index);

        if (recording_index == start_recording_index) {
          // TODO: Maybe knowing this, we can make something to prevent it.
          ROS_INFO("---left(limit of recording)---");
        } else {
          ROS_INFO("---left---");
        }
      }

    } else if (already_in_voice == 1) {
      prob_voice = rnnoise_process_frame(st, frame_values_float, frame_values_float);

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
      prob_voice = rnnoise_process_frame(st, frame_values_float, frame_values_float);

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

void onAudioCallback(const audio_common_msgs::AudioData::ConstPtr msg){
  // After checking the correct number of elements in the msg, cast in the easier
  // way the uint8_t to int16_t array using the underlaying array of the vector.
  if (msg->data.size() == NUMBER_FRAMES_RNNOISE * (sizeof(int16_t) / sizeof(uint8_t))) {
    RNNoiseProcessNewInput((int16_t*)msg->data.data());
  } else {
    ROS_INFO("Error in the number of frames received: %zu.", msg->data.size());
  }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "InputAudio");
    ros::NodeHandle n;
    ROS_INFO("*Node Started*");

    cout << "NUM_ITERATIONS_ALMOST_IN_VOICE= " << NUM_ITERATIONS_ALMOST_IN_VOICE << endl;
    cout << "MAX_ITERATIONS_WITHOUT_VOICE= " << MAX_ITERATIONS_WITHOUT_VOICE << endl;


    st = rnnoise_create(nullptr);
    RNNoiseWarmUp(st);
    ROS_INFO("*RNNoise ready*");


    publi = n.advertise<audio_common_msgs::AudioData>("UsefulAudio", ADVERTISE_BUFFER_SIZE);
    
    ros::Subscriber sub = n.subscribe("rawAudioChunk", 5, onAudioCallback);
    
    ros::spin();


    rnnoise_destroy(st);
    ROS_INFO("*RNNoise destroyed, leaving...*");

    return 0;
}
