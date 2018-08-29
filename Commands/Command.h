#ifndef COMMAND_H_INCLUDED
#define COMMAND_H_INCLUDED
#include <string>

using namespace std;

class Command{
public:
	//Movimientos y acciones
	void goTo(string destination);
	void go();
	void followPerson();
	void take();
	void carryToLocation();
	void putDown();
	//sets
	void setDestination(string destination);
	void setMainVerb(string mainVerb);
	//gets
	bool isItBusy();
	//helper methods
	string getDestination();
	string getMainVerb();

private:
	string destination;
	string mainVerb;
	bool isBusy;


};

void Command::goTo(string destination){
	//Accion de ir a un lugar
	this.destination= destination;

}
void Command:: goTo(){
	
}
#endif // COMMAND_H_INCLUDED
