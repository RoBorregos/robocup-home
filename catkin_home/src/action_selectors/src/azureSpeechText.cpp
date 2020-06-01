/**
 *  SpeechAPI ROS Node
 *
 * Note that when internet is kind of failing it can take almost
 * 10 seconds to output error and abort request.
 */
#include <ros/ros.h>
#include <cctype>
#include <iostream>
#include <speechapi_cxx.h>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include "audio_common_msgs/AudioData.h"
#include "action_selectors/RawInput.h"

using namespace std;
using namespace Microsoft::CognitiveServices::Speech;
using namespace Microsoft::CognitiveServices::Speech::Audio;

#include "AudioInputBytes.h"
#include "SpeechAPIUtils.h"
SpeechAPIUtils SpeechUtils;

ros::Publisher publi;


void onAudioCallback(const audio_common_msgs::AudioData::ConstPtr msg){
    
    ROS_INFO_STREAM("AUDIO RECEIVED");
    
    // Creates an instance of a speech config with specified subscription key and service region.
    auto config = SpeechConfig::FromSubscription(SpeechUtils.getAPI(), SpeechUtils.getRegion());

    // Creates a speech recognizer using file as audio input.
    auto callback = make_shared<AudioInputBytes>(msg->data);
    
    
    auto pullStream = AudioInputStream::CreatePullStream(AudioStreamFormat::GetWaveFormatPCM(16000, 16, 1), callback);
    auto audioInput = AudioConfig::FromStreamInput(pullStream);
    
    auto recognizer = SpeechRecognizer::FromConfig(config, audioInput);
    
    // Starts speech recognition, and returns after a single utterance is recognized. The end of a
    // single utterance is determined by listening for silence at the end or until a maximum of 15
    // seconds of audio is processed.
    auto result = recognizer->RecognizeOnceAsync().get();

    // Checks result.
    if (result->Reason == ResultReason::RecognizedSpeech)
    {
        if (std::all_of(result->Text.begin(), result->Text.end(), ::isspace)) {
            ROS_INFO("Text recognized is empty");
            return;
        }

        ROS_INFO_STREAM("Recognized: " << result->Text);
        //Publish messge
        action_selectors::RawInput msg;
        msg.isWoman = false;
        msg.inputText = result->Text;
        publi.publish(msg);
    }
    else if (result->Reason == ResultReason::NoMatch)
    {
        ROS_INFO_STREAM("NOMATCH: Speech could not be recognized.");
    }
    else if (result->Reason == ResultReason::Canceled)
    {
        auto cancellation = CancellationDetails::FromResult(result);
        ROS_INFO_STREAM("CANCELED: Reason=" << (int)cancellation->Reason);

        if (cancellation->Reason == CancellationReason::Error) 
        {
            ROS_INFO_STREAM("CANCELED: ErrorCode= " << (int)cancellation->ErrorCode);
            ROS_INFO_STREAM("CANCELED: ErrorDetails=" << cancellation->ErrorDetails);
            ROS_INFO_STREAM("CANCELED: Did you update the subscription info?");
        }
    }
}



int main(int argc, char* argv[]){

  // This must be called before anything else ROS-related
  ros::init(argc, argv, "AzureSpeechToText");
  
  ROS_INFO_STREAM("*Node initiated*");
  
  // Create a ROS node handle
  ros::NodeHandle nh;
  publi = nh.advertise<action_selectors::RawInput>("RawInput", 10);
  ros::Subscriber sub = nh.subscribe("UsefulAudio16kHZ", 5, onAudioCallback);
  
  // Don't exit the program.
  ros::spin();
}