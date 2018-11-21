#include <string>
#include <iostream>
#include "Command.h"
#include <unordered_map>
#include <vector>
#include <sstream>
#include <fstream>
using namespace std;
/*
TO DO:

multiple tags of the same type in a sentence (bring me the apple and the soda from the kitchen). DONE
new command: save me, remember my face DONE
questions: who is here, is it diego here,   PARTIALLY DONE
Feed the dictionary with more words 
Command.h 

*/



//Very global and very important variables lol sorry
bool debug=true;
unordered_map<string,string> tags;

//Check if there is an 'and' joining 2 sentences based on the quantity of verbs
bool checkAnd(unordered_map<string,vector<string>> sentenceTags,unordered_map<string,vector<string>> ::iterator end){
	cout<<"--Checks if multiple sentences "<<endl;
	unordered_map<string,vector<string>>::iterator it;
	int  mainVerbs=0;
	for(it=sentenceTags.begin(); it!= end; it++){
		string first  = it->first;
		vector<string> second = it->second;
		//If more than 1 main verb, 2 sentences
		if(first=="VG" || first=="VGO" || first=="VS" ||first=="VO"){
			mainVerbs++;
			if(second.size()>1){
				return true;
			}
		}
		if(mainVerbs>1){
			return true;
		}
	}
	return false;
}

string auxiliarName(unordered_map<string,vector<string>> sentenceTags, string verb){
	for(auto it=sentenceTags["UW"].begin(); it!=sentenceTags["UW"].end(); it++){
		if(*it=="name"){
			cout<<"name is type of"<<endl;
			if(sentenceTags["UW"].size()>=2){
				return sentenceTags["UW"][1];
			}
			else{
			cout<<"No name given"<<endl;
			}
		
		}
		else if(verb!="is"){
			cout<<"im type of sentence"<<endl;
			return sentenceTags["UW"][0];
		}

	}
	return "";
}

//-----------------------------Switch con los verbos conocidos, brutalmente sencillo----------------
bool addToActions(Command &MainActions, unordered_map<string,vector<string>> sentenceTags){
	cout<<"--Ready to generate commands from sentences"<<endl;
	unordered_map<string,vector<string>> ::iterator end = sentenceTags.end();
 	bool multipleSentences = checkAnd(sentenceTags,end);
 	if(multipleSentences){
		 cout<<"--Multiple sentences identified "<<endl;
 		return false;
 	}
	//For questions
	if(sentenceTags.find("QW")!=end){
		for(auto& it : sentenceTags["QW"]){
			if(it=="who"){
				cout<<"Detect faces"<<endl;
				return true;
			}
		}
	}
	//A special case for each type of verb
	if(sentenceTags.find("VS")!=end){
		 //HARDCODED ACTIONS :(
		 for(auto& it : sentenceTags["VS"]){
			 //Save faces
			 string sName;
			 if(it=="remember" || it=="save"){
				 cout<<"Please give me your name"<<endl;
				 cin>>sName;

			 }
			 else if(it=="am" || it=="is" || it=="im"){
				 cout<<"About to call auxiliarName"<<endl;
				  sName = auxiliarName(sentenceTags,it);
				
			 }
			if(sName.length()>0){
				cout<<"I must save the face of the person talking to me"<<endl;
				cout<<sName<<endl;
				//llamar save face
				return true;
			}
			 
			//Detect faces
			if(it=="detect"){
				cout<<"Detect faces";
				return true;
			}
		 }
		 
	 }
	 else if(sentenceTags.find("VE")!=end){
		 cout<<"ORB SLAM"<<endl;
	 }
	 else if(sentenceTags.find("VG")!=end){
		 if(sentenceTags.find("LO")!=end){
			 for(auto it=sentenceTags["LO"].begin(); it!=sentenceTags["LO"].end(); it++){
				 cout<<"I have to "<<sentenceTags["VG"][0]<<" to the "<<*it<<endl; 
				 MainActions.goTo(*it);
			 }
			 	
		 }
		 else{
			 cout<<"--Sorry no destination given, cant complete task"<<endl;
		 }
	 }
	 else if(sentenceTags.find("VO")!=end){
		 if(sentenceTags.find("KO")!=end || sentenceTags.find("KP")!=end){
			 //Classify people as "Known objects", if no other KO in sentence
			 //Here are different possible actions (take,putDown,bring,lift)
			 if(sentenceTags.find("LO")!=end){
				 //"take this somewhere" type of command 
				 /*
				 TO DO: check if person or object  <-----
				 */
				for (auto& it : sentenceTags["KO"]) {
    				cout<<"I have to "<<sentenceTags["VO"][0]<<" the "<<it<<" to the"<<sentenceTags["LO"][0]<<endl;
				}
				
				for (auto& it : sentenceTags["KP"]) {
    				cout<<"I have to "<<sentenceTags["VO"][0]<<" the "<<it<<" to the "<<sentenceTags["LO"][0]<<endl;
				}
			 }
			 else{
				 
				for (auto& it : sentenceTags["KO"]) {
    				cout<<"I have to "<<sentenceTags["VO"][0]<<" the "<<it<<endl;
				}
				 
			 }
		 }
		 else{
			 cout<<"--Sorry no object or person given, cant complete task"<<endl;
		 }
	 }
	 cout<<"--Finished processing this command"<<endl;
	 return true;
}

//----------------------------------------------------------------------------

void readDictionary(){
	//Loads the data from dictionary.txt into the hashmap
	ifstream dataFile("dictionary.txt");
	if (!dataFile.is_open()) {
		 // The file hasn't been opened; take appropriate actions here.
		 throw  "File could not be opened";
	 }
	 string key,val;
	 //cout<<"Whole dictionary: "<<endl;
	 while(dataFile>>key>>val){
		 tags[key]=val;
		 if(false){
			 cout<<key<<": "<<val<<endl;
		 }
	 }
	 dataFile.close();
}

string tokenize(string word){
	for(int i=0; i<int(word.length()); i++){
		if(word[i] <= 'Z' && word[i] >= 'A'){
			//LowerCase
			word[i] = word[i] - ('Z' - 'z');
		}
		if((int(word[i])<97 || int(word[i])>122) && int(word[i])!=32){
			// Only valid chars [a,z]
			word = word.erase(i,1);

		}
	}
	return word;
}

string splitIntoSentences(string &newSentence,string sentence){
	cout<<"--Splits sentence in two"<<endl;
	for(int i=0; i<sentence.length()-2; i++){
		if(sentence[i]=='a'&&sentence[i+1]=='n'&&sentence[i+2]=='d'){
			int endSen = sentence.length()-(i+3);
			newSentence = sentence.substr(i+3,endSen);
			sentence = sentence.substr(0,sentence.length()-endSen-3);
			return sentence;
		}
	}
}

void parseSentence(vector<string> &sentences, Command &MainActions, int i){
	string sentence = sentences[i];
	cout<<"------------New sentence to be parsed-------------"<<endl;
	//Regresa el MainActions Modificado con las nuevas acciones agregadas al queue.
	cout<<tokenize(sentence)<<endl;
	stringstream ss;
	ss<<sentence;
	string word;
	string code;
	unordered_map<string,vector<string>> sentenceTags;
	while(ss>>word){
		word = tokenize(word);
		if(tags.find(word)!=tags.end()){
			//Checks in the dictionary if word exists and links the corresponding tag
			code=tags[word];
			sentenceTags[code].push_back(word);
		}
		else{
			sentenceTags["UW"].push_back(word);
		}
	}
	if(debug){
		cout<<"Tags currently identified in this sentence:"<<endl;
		unordered_map<string,vector<string>>::iterator it;
		for(it=sentenceTags.begin(); it!= sentenceTags.end(); it++){
			cout<<it->first;
			for(int r =0; r < it->second.size();r++){
				cout<<" "<<it->second[r];
			}
			cout<<endl;
		}
	}
	 
	if(!addToActions(MainActions, sentenceTags)){
		string newSentence;
		//Splits the current sentence into 2 sentences
		sentences[i]=splitIntoSentences(newSentence, sentences[i]);
		//Adds it to the list of commands
		cout<<newSentence<<endl;
		sentences.insert(sentences.begin()+i+1,newSentence);
		//Recursive call to process again the current sentence
		cout<<"--Checks again the sentence"<<endl;
		parseSentence(sentences,MainActions,i);
	}
	//Identify corresponding action on the robot
	//Iterate through sentenceTags and check if first[0]=='V' then a verb has been detected

}

int main(){
	//Open file
	//Generate hash map from text file

	Command MainActions;
	vector<string> testSentences;
	readDictionary();
	//Hacer el llamado al Alexa API
	
	//Recibir el string
	cout<<"Current sentence:  "<<endl;
	//string testSentence = "roBot, help. me take. ,the apples and the soda";
	//testSentences.push_back("example and second");
	//testSentences.push_back(testSentence);

	//testSentences.push_back("Go to the KiTcHen");
	//testSentences.push_back("robot, take me to the KiTcHen with Diego and bring apples");
	//testSentences.push_back("Robot save my face");
	testSentences.push_back("Robot remember my face");
	testSentences.push_back("Robot save my face");
	testSentences.push_back("My name is Paul");
	testSentences.push_back("Hello I'm Paul");
	testSentences.push_back("Hello my name Paul");
	testSentences.push_back("Who is here?");
	testSentences.push_back("Detect who I am");
	testSentences.push_back("Who am I?");
	testSentences.push_back("Who is in this picture");
	testSentences.push_back("Robot explore the room");
	for(int i=0; i<testSentences.size();i++){
		//cout<<testSentences[i]<<endl;
		parseSentence(testSentences, MainActions,i);
	}
}
