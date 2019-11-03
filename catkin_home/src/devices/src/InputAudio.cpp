#include "ros/ros.h"
#include "audio_common_msgs/AudioData.h"
#include <vector>
#define BUFFER_SIZE 256
using namespace std;
ros::Publisher publi;

void onAudioCallback(const audio_common_msgs::AudioData::ConstPtr msg){
    cout<<"I heard "<<endl;
    vector<uint8_t> test;
    test.push_back(1);
   //Define otro mensaje que va a enviar
    audio_common_msgs::AudioData OutMsg;
    OutMsg.data = msg->data;
    publi.publish(OutMsg);
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
