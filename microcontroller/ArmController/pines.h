#include <Servo.h> 

#define pi 3.14159265358979323846 
#define dirPinM1 24
#define stepPinM1 23
#define enPinM1 22
#define dirPinM4 27
#define stepPinM4 26
#define enPinM4 25
#define dirPinM7 29
#define stepPinM7 36
#define enPinM7 28
#define dirPinM8 41
#define stepPinM8 39
#define enPinM8 40
#define to_grad(rad) ((rad*180.0)/pi) 
#define to_rad(grad) ((grad*pi)/180.0)
#define nemaStepMts 0.02 //test or calculate...
#define limitSwitch2 7 // es el de elevador brazo 2
#define limitSwitch1 6 
#define limitSwitch3 14
#define delayBetweenSteps 100

Servo garraI;  
Servo garraD;  
Servo munecaV;  
Servo cuello;

void declaraPines(){
   // input
   pinMode(limitSwitch2, INPUT_PULLUP);
   pinMode(limitSwitch1, INPUT_PULLUP);
   pinMode(limitSwitch3, INPUT_PULLUP);
   
   // Marcar los pines como salida
   pinMode(dirPinM1, OUTPUT);
   pinMode(stepPinM1, OUTPUT);
   pinMode(enPinM1, OUTPUT);
   digitalWrite(enPinM1, LOW);
   pinMode(dirPinM4, OUTPUT);
   pinMode(stepPinM4, OUTPUT);
   pinMode(enPinM4, OUTPUT);
   digitalWrite(enPinM4, LOW);
   pinMode(dirPinM7, OUTPUT);
   pinMode(stepPinM7, OUTPUT);
   pinMode(enPinM7, OUTPUT);
   digitalWrite(enPinM7, LOW);
   pinMode(dirPinM8, OUTPUT);
   pinMode(stepPinM8, OUTPUT);
   pinMode(enPinM8, OUTPUT);
   digitalWrite(enPinM8, LOW);
   
   garraI.attach(44);  // vincula el servo al pin digital 9
   garraD.attach(45);  // vincula el servo al pin digital 9
   munecaV.attach(46);  // vincula el servo al pin digital 9
   // cuello.attach(47); 
}

void delayZ(long long ms){
  unsigned long long now = millis();
  
  while(millis() - now < ms){
    
  }
}

void delayZMicro(long long micro){
  unsigned long long now = micros();
  
  while(micros() - now < micro){
    
  }
}
