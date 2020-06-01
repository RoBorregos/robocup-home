#ifndef SPEECH_API_UTILS_H
#define SPEECH_API_UTILS_H

#include "ros/package.h"

class SpeechAPIUtils {
    public:
        SpeechAPIUtils();
        void getGlobalVars();
        string getAPI();
        string getRegion();

    private:
        string API="";
        string Region="";
        const string VarsFile = 
            ros::package::getPath("action_selectors") + "/data/AzureSTTAPIKey.txt";
};


SpeechAPIUtils::SpeechAPIUtils(){
    getGlobalVars();
}

void SpeechAPIUtils::getGlobalVars(){
    ifstream infile;
    infile.open (this->VarsFile);
    int count=1;
    string var;
    while(!infile.eof()){
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
    infile.close();
}

string SpeechAPIUtils::getAPI(){
    return this->API;
}

string SpeechAPIUtils::getRegion(){
    return this->Region;
}

#endif
