#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

#include "pines.h"
#include "nema.h"
#include "torso.h"
#include "elevador.h"
#include "munecaH.h"
#include "codo.h"


unsigned long long last_callback;
int callback_counter;
char log_msg[20];
char num[8];
ros::NodeHandle nh;

#include "servos.h"
#include "linked.h"


bool moveFlag = true;

void rosLog(const sensor_msgs::JointState& cmd_msg){
  callback_counter++;
  sprintf(log_msg,"Callback Counter: %d", callback_counter);
  nh.loginfo(log_msg);
  
  sprintf(log_msg,"Time: %d", last_callback);
  nh.loginfo(log_msg);
  
  int arrSize = cmd_msg.position_length;
    
  if(arrSize == 5){ // arm_group
  
    dtostrf(cmd_msg.position[0], 6, 4, num);
    sprintf(log_msg, "Muneca V: %s", num);
    nh.loginfo(log_msg);
    if(moveFlag == true)
    moveMunecaV(cmd_msg.position[0]);
    
    dtostrf(cmd_msg.position[1], 6, 4, num);
    sprintf(log_msg, "Muneca H: %s", num);
    nh.loginfo(log_msg);
    if(moveFlag == true)
    moveMunecaH(cmd_msg.position[1]);
    
    dtostrf(cmd_msg.position[2], 6, 4, num);
    sprintf(log_msg, "Torso: %s", num);
    nh.loginfo(log_msg);
    if(moveFlag == true)
    moveTorso(cmd_msg.position[2]);
    
    dtostrf(cmd_msg.position[3], 6, 4, num);
    sprintf(log_msg, "Elevador: %s", num);
    nh.loginfo(log_msg);
    
    dtostrf(cmd_msg.position[4], 6, 4, num);
    sprintf(log_msg, "Codo: %s", num);
    nh.loginfo(log_msg);
    if(moveFlag == true)
    moveCodo(cmd_msg.position[4]);
  
  }else if(arrSize ==2){ // gripper
    dtostrf(cmd_msg.position[0], 6, 4, num);
    sprintf(log_msg, "Garra D: %s", num);
    nh.loginfo(log_msg);
    if(moveFlag == true)
    moveGarraD(cmd_msg.position[0]);
    dtostrf(cmd_msg.position[1], 6, 4, num);
    sprintf(log_msg, "Garra I: %s", num);
    nh.loginfo(log_msg);
    if(moveFlag == true)
    moveGarraI(cmd_msg.position[1]);
  }else if(arrSize == 1){ // neck
    dtostrf(cmd_msg.position[0], 6, 4, num);
    sprintf(log_msg, "Neck: %s", num);
    nh.loginfo(log_msg);
    
  } 
  last_callback = millis();
}


ros::Subscriber<sensor_msgs::JointState> sub("move_group/fake_controller_joint_states", &rosList);
//ros::Subscriber<sensor_msgs::JointState> sub("move_group/fake_controller_joint_states", &rosCallback);

void goToOrigin(){
  munecaV.write(180);
  delay(50);
  garraI.write(garraIG);
  garraD.write(garraDG);
  goToOriginTorso();
  goToOriginCodo();
  goToOriginMunecaH();
  munecaV.write(munecaVG);
}

void setup() {
  // put your setup code here, to run once:

  callback_counter = 0;
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  declaraPines();
  goToOrigin();
  nh.loginfo("SETUP");
  last_callback = millis();
  listSize = 0;
}

void loop() {
  //testServo(garraI);
  //testServo(garraD);
  //testServo(munecaV);
  //testNema(dirPinM1, stepPinM1);
  //testClose();
  nh.spinOnce();
  if(millis() - last_callback > 4000 && listSize>1){
    executeCommands(head);
    deleteList();
  }
}
