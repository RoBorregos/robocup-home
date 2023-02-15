// ARM [-1,0.5] (up, down)
double munecaVR = 0; //S3
int munecaVG = 0; // to test [0 arriba 

//GRIPPER

double garraDR = 0; // S2
int garraDG = 120;
double garraIR = 0; // S1
int garraIG = 50; // [0 izq mas derecha]

// NECK

double cuelloVG = 120; // S7 [-0.8, 0.5 arriba,abajo)
int cuelloVR = 0;

void testServo(Servo s){

  for(int i=0; i<=120; i++){
    s.write(i);
    delay(20);
  }
  delay(500);
  for(int i=120; i>=0; i--){
    s.write(i);
    delay(20);
  }
  
}

double constraint(double g){
  if(g<0){
    return 0;
  }else if(g>180){
    return 180;
  }
  return g;
}
// ARM [-1,0.5] (up, down)

void testClose(){
  garraD.write(180);
  garraI.write(0);

  for(int i=0; i<=90; i++){
    garraI.write(i);
    garraD.write(180-i);
    delay(30);
  }

  delay(4000);
}

void moveMunecaV(double rad){
  double actual = munecaVR;
  munecaVR = rad;
  double deltaRad = abs(rad - actual);
  double deltaGrad = to_grad(deltaRad)*3.5;
  bool dir = actual < rad ? 1 : 0;
  if(dir == 1){
    munecaVG+=deltaGrad;
  }else{
    munecaVG-=deltaGrad;
  }
  //munecaVG = constraint(munecaVG);
  munecaV.write(munecaVG);
  sprintf(log_msg, "munecaVG: %d", munecaVG);
    nh.loginfo(log_msg);
  delay(10);
}

void moveGarraI(double rad){
  double actual = garraIR;
  garraIR = rad;
  double deltaRad = abs(rad - actual);
  double deltaGrad = to_grad(deltaRad);
  bool dir = rad > actual ? 0 : 1;
  if(dir == 1){
    garraIG+=deltaGrad;
  }else{
    garraIG-=deltaGrad;
  }
  garraIG = constraint(garraIG);
  garraI.write(garraIG);
  delay(10);
}

void moveGarraD(double rad){
  double actual = garraDR;
  garraDR = rad;
  double deltaRad = abs(rad - actual);
  double deltaGrad = to_grad(deltaRad);
  bool dir = rad > actual ? 0 : 1;
  if(dir == 1){
    garraDG+=deltaGrad;
  }else{
    garraDG-=deltaGrad;
  }
  garraDG =constraint(garraDG);
  garraD.write(garraDG);
}

void moveCuello(double rad){
   double actual = cuelloVR;
  cuelloVR = rad;
  double deltaRad = abs(rad - actual);
  double deltaGrad = to_grad(deltaRad);
  bool dir = rad > actual ? 1 : 0;
  if(dir == 1){
    cuelloVG+=deltaGrad;
  }else{
    cuelloVG-=deltaGrad;
  }
  cuelloVG =constraint(cuelloVG);
  cuello.write(cuelloVG);
  delay(10);
}
