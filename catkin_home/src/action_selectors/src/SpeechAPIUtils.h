
class SpeechAPIUtils {
    public:
        SpeechAPIUtils();
        void getGlobalVars();
        string getAPI();
        string getRegion();


    private:
        string API="";
        string Region="";
        string VarsFile="src/action_selectors/src/GLOBAL.txt";
  
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