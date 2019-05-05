#include "ros/ros.h"
#include "audio_common_msgs/AudioData.h"


using namespace std;


#define BUFFER_SIZE 256
constexpr int NUMBER_FRAMES_RNNOISE = 480L;


ros::Publisher publi;


void rnnoise_process_new_input(uint16_t new_input[NUMBER_FRAMES_RNNOISE]) {}

void onAudioCallback(const audio_common_msgs::AudioData::ConstPtr msg){
    static uint16_t buffer_input_rnnoise[NUMBER_FRAMES_RNNOISE];
    static int index_buffer_rnnoise = 0;


    buffer_input_rnnoise[index_buffer_rnnoise] = ((uint16_t*)msg->data)[0];    

    index_buffer_rnnoise = 
        (index_buffer_rnnoise + 1) % NUMBER_FRAMES_RNNOISE;

    if (index_buffer_rnnoise == 0) {
        rnnoise_process_new_input(buffer_input_rnnoise);
    }


    ros::Rate loop_rate(10);
    loop_rate.sleep();
}


int main(int argc, char **argv){
    ros::init(argc,argv,"InputAudio");
    
    
    ros::NodeHandle n;
    publi = n.advertise<audio_common_msgs::AudioData>("UsefulAudio", BUFFER_SIZE);
    
    ros::Subscriber sub = n.subscribe("audio", 1000, onAudioCallback);
    
    
    ros::spin();


    return 0;
}
