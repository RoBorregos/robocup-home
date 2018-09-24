#include <string>
#include <iostream>
#include "Command.h"
#include <unordered_map>
#include <vector>
#include <sstream>
#include <fstream>
using namespace std;

//Very global and very important variables
bool debug=true;
unordered_map<string,string> tags;

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
		if(int(word[i])<97 || int(word[i])>122){
			// Only valid chars [a,z]
			word = word.erase(i,1);

		}
	}
	return word;
}

void parseSentence(string sentence, Command &MainActions){
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
			code=tags[word];
			sentenceTags[code]=word;
			//MainActions.addAction(code);
		}
	}

	if(debug){
		cout<<"tags currently identified in this sentence"<<endl;
		unordered_map<string,string>::iterator it;
		for(it=sentenceTags.begin(); it!= sentenceTags.end(); it++){
			cout<<it->first<<" "<<it->second<<endl;
		}
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
	string testSentence = "roBot, help. me carrying. ,these  bags to THE KitChen";
	testSentences.push_back(testSentence);
	parseSentence(testSentences[0], MainActions);




}
