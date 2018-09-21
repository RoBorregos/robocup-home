#include <string>
#include <iostream>
#include "Command.h"
#include <unordered_map>
#include <vector>
#include <sstream>
using namespace std;


void MainActions(string sentence, Command &MainActions){
	//Regresa el MainActions Modificado con las nuevas acciones agregadas al queue.
	sstream ss;
	ss<<sentence;
	string word;
	
	while(ss>>word){
		cout<<word<<endl;
	}

}
int main(){
	//Hacer el llamado al Alexa API
	//Recibir el string
	Command MainActions;
	vector<string> testSentences;
	string testSentence = "Robot ayudame a ccargar estas bolsas";
	testSentences.push_back(testSentence);
	parseSentence(testSentence[0], MainActions);




}
