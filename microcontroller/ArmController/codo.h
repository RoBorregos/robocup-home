
double codoR = 0; // M1

int stepsInitC = 10000; // NEEDS TEST
double stepPerRadC = stepsInitC / (2*pi/3.0);

// dir = 0 is NEEDS TEST
void goToOriginCodo(){
  bool dir = 0;
  while(digitalRead(limitSwitch2) == HIGH){
      nemaStep(dirPinM1, stepPinM1, dir);
  }
  for(int i=0; i<stepsInitC; i++){
    nemaStep(dirPinM1, stepPinM1, !dir);
  } 
}

void moveCodo(double rad){
  double codoAcR = codoR;
  codoR = rad; // set current pos tu new

  bool dir = codoR < codoAcR ? 0 : 1; // menor -> clockwise
  int stepsToGoal = stepPerRadC * abs(codoR-codoAcR);
  
  for(int i=0; i<stepsToGoal; i++){
    nemaStep(dirPinM1, stepPinM1, dir);
  }
}
  
