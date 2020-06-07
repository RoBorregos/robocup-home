#ifndef SPEECH_API_UTILS_H
#define SPEECH_API_UTILS_H

#include "ros/package.h"
#include <ros/ros.h>

// TODO: Making this a singleton could be better idea.
class SpeechAPIUtils {
    public:
        SpeechAPIUtils() {}
        // Reads the file to get the values; only in success
        // returns true.
        bool init();
        string getAPI();
        string getRegion();

    private:
        string API="";
        string Region="";
        const string VarsFile = 
            ros::package::getPath("action_selectors") + "/data/AzureSTTAPIKey.txt";
};


bool SpeechAPIUtils::init(){
    ifstream infile(this->VarsFile);
    if (!infile.is_open() || infile.fail()) {
        ROS_INFO("ERROR: Error opening file: %s", this->VarsFile.c_str());
        return false;
    }

    int count=1;
    string var;
    while(infile.good()){
        getline(infile,var); 
        
        switch (count){
        case 1:
            API=var;
            break;
        case 2:
            Region=var;
            break;
        }
        count++;
    }

    if (count < 3) {
        ROS_INFO("ERROR: couldn't get two lines in the file.");
        return false;
    }

    infile.close();
    return true;
}

string SpeechAPIUtils::getAPI(){
    return this->API;
}

string SpeechAPIUtils::getRegion(){
    return this->Region;
}

#endif
