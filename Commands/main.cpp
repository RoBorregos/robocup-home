#include <string>
#include <iostream>
#include "Command.h"
#include <unordered_map>
#include <vector>
#include <sstream>
#include <fstream>
using namespace std;

//Very global and very important variables lol sorry
bool debug=true;
unordered_map<string,string> tags;
//-----------------------------Switch con los verbos conocidos, brutalmente sencillo----------------
void addToActions(Command &MainActions, unordered_map<string,string> sentenceTags){
	unordered_map<string,string> ::iterator end = sentenceTags.end();
 if(sentenceTags.find("VS")!=end){
	 //HARDCODED ACTIONS :(
	 return;
 }
 else if(sentenceTags.find("VG")!=end){
	 if(sentenceTags.find("LO")!=end){
		 	cout<<"I have to "<<sentenceTags["VG"]<<" to the "<<sentenceTags["LO"]<<endl;
			MainActions.goTo(sentenceTags["LO"]);
	 }
	 else{
		 cout<<"Sorry no destination given, cant complete task"<<endl;
	 }
 }
 else if(sentenceTags.find("VO")!=end){
	 if(sentenceTags.find("KO")!=end){
		 //Here different possible actions (take,putDown,bring,lift)
		 if(sentenceTags.find("LO")!=end){
			 //"take this somewhere" type of command
			  cout<<"I have to "<<sentenceTags["VO"]<<" the "<<sentenceTags["KO"]<<"to "<<sentenceTags["LO"]<<endl;
		 }
		 else{
			 cout<<"I have to "<<sentenceTags["VO"]<<" the "<<sentenceTags["KO"]<<endl;
		 }
	 }
	 else{
		 cout<<"Sorry no object given, cant complete task"<<endl;
	 }
 }
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

void parseSentence(string sentence, Command &MainActions){
	cout<<"------------New sentence to be parsed-------------"<<endl;
	//Regresa el MainActions Modificado con las nuevas acciones agregadas al queue.
	cout<<tokenize(sentence)<<endl;
	stringstream ss;
	ss<<sentence;
	string word;
	string code;
	unordered_map<string,string> sentenceTags;
	while(ss>>word){
		//cout<<word<<endl;
		word = tokenize(word);
		//cout<<word<<endl;
		if(tags.find(word)!=tags.end()){
			//Checks in the dictionary if word exists and links the corresponding tag
			if(word=="and"){
				cout<<"Check if conjunction between sentences "<<endl;
			}
			code=tags[word];
			sentenceTags[code]=word;
		}
		else{
			cout<<"I don't know the following word"<<word<<endl;
		}
	}
	if(debug){
		cout<<"tags currently identified in this sentence"<<endl;
		unordered_map<string,string>::iterator it;
		for(it=sentenceTags.begin(); it!= sentenceTags.end(); it++){
			cout<<it->first<<" "<<it->second<<endl;
		}
	}
	addToActions(MainActions, sentenceTags);
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
	string testSentence = "roBot, help. me carrying. ,these  bags to THE KitChen";
	testSentences.push_back(testSentence);
	testSentences.push_back("Go to the KiTcHen");
	for(int i=0; i<testSentences.size();i++){
		parseSentence(testSentences[i], MainActions);
	}
}
