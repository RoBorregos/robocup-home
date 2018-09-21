#include <string>
#include <iostream>
#include "Command.h"
#include <unordered_map>
#include <vector>
#include <sstream>
using namespace std;

void readDictionary(){

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
	while(ss>>word){
		//cout<<word<<endl;
		cout<<word<<endl;
		word = tokenize(word);
		cout<<word<<endl;
		//code = lookupWord(word);
		if(code!=""){
			//MainActions.addAction(code);
		}
	}
}

int main(){
	//Open file
	//Generate hash map from text file
	//Hacer el llamado al Alexa API
	//Recibir el string
	Command MainActions;
	vector<string> testSentences;
	unordered_map<string,string> tags;
	string testSentence = "roBot, help. me carrying. ,these  bags to THE KitChen";
	testSentences.push_back(testSentence);
	parseSentence(testSentences[0], MainActions);




}
