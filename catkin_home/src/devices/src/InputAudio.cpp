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
#include "RnnoiseMethods/DetectionAlgorithms.hpp"
#include "RnnoiseMethods/Algorithm3Steps1.hpp"


using namespace std;
using namespace rnnoise_methods;
using namespace rnnoise_methods::detection_algorithms;


constexpr int ADVERTISE_BUFFER_SIZE = 10;

constexpr bool PUBLISH_WITHOUT_NOISE = true;


const RnnoiseNewInputProcessor rnnoise_process_new_input = Algorithm3Steps1ProcessNewInput;

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

/**
 * This is of type `RnnoiseProcessor`.
 */
float RNNoiseProcessFrames(const float in_frames[NUMBER_FRAMES_RNNOISE], 
  float out_frames[NUMBER_FRAMES_RNNOISE]) {
  return rnnoise_process_frame(st, out_frames, in_frames);
}

/*
 * This is of type `AudioPublisher`.
 */
void PublishAudio(const int16_t* const recording, 
  const long array_length, const long begin_index_element, 
  const long end_index_element) {
  vector<uint8_t> output_vector;

  if (begin_index_element < end_index_element ) {
    // The recording doesn't go circular.
    output_vector.assign(
      (uint8_t*)(recording + begin_index_element),
      (uint8_t*)(recording + end_index_element));
  } else {
    // The recoding goes circular and we have to save it in two steps.
    const long length_int8_elements = 
      (array_length - begin_index_element + end_index_element) 
      * sizeof(int16_t);
    output_vector.reserve(length_int8_elements);

    output_vector.insert(output_vector.end(),
      (uint8_t*)(recording + begin_index_element),
      (uint8_t*)(recording + array_length));
    output_vector.insert(output_vector.end(), 
      (uint8_t*)recording,
      (uint8_t*)(recording + end_index_element));
  }

  audio_common_msgs::AudioData msg_audio;
  msg_audio.data = output_vector;

  publi.publish(msg_audio);
}

void onAudioCallback(const audio_common_msgs::AudioData::ConstPtr msg){
  // After checking the correct number of elements in the msg, cast in the easier
  // way the uint8_t to int16_t array using the underlaying array of the vector.
  if (msg->data.size() == NUMBER_FRAMES_RNNOISE * (sizeof(int16_t) / sizeof(uint8_t))) {
    rnnoise_process_new_input((int16_t*)msg->data.data(), RNNoiseProcessFrames,
      PublishAudio, PUBLISH_WITHOUT_NOISE);
  } else {
    ROS_INFO("Error in the number of frames received: %zu.", msg->data.size());
  }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "InputAudio");
    ros::NodeHandle n;
    ROS_INFO("*Node Started*");


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
