#ifndef COMMAND_H_INCLUDED
#define COMMAND_H_INCLUDED
#include <string>
#include <queue>

using namespace std;

class Command{
public:
	//Movimientos y acciones
	void goTo(string destination);
	void go();
	void followPerson();
	void followPerson(string);
	void stop();
	void take(string);
	void carryToLocation(string);
	void putDown();
	//sets
	void setDestination(string destination);
	void setMainVerb(string mainVerb);
	void setMaster(string);
	void addAction(string);
	//gets
	bool isItBusy();
	string getMaster();
	//helper methods
	string getDestination();
	string getMainVerb();

private:
	queue<char> PendingTasks;
	string destination;
	string mainVerb;
	bool isBusy;
	string master;


};

void Command::goTo(string destination){
	//Accion de ir a un lugar
	this->destination= destination;

}


#endif // COMMAND_H_INCLUDED
