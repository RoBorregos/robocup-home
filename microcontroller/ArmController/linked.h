struct node {
  int group; // 0 arm, 1 gripper, 2 neck
  int id;
  double a;
  double b;
  double c;
  double d;
  double e;
  node *next;
};

node *head;
size_t listSize;

// empujar elemento
void pushList(int group, double a, double b, double c, double d, double e, int id)
{
  if(listSize == 0){
    listSize = 1;
    head = new node();
    head->next = nullptr;
    head->group = group;
    head->a = a;
    head->b = b;
    head->c = c;
    head->d = d;
    head->e = e;
    head->id = id;
  }else{
    listSize++;
    node *newHead = new node();
    newHead->next = head;
    newHead->group = group;
    newHead->a = a;
    newHead->b = b;
    newHead->c = c;
    newHead->d = d;
    newHead->e = e;
    newHead->id = id;
    head = newHead;
  }
}

void executeCommands(node *list){
  if(listSize == 0) 
    return;
  if(list->next != nullptr)
    executeCommands(list->next);

   int group = list->group;

   sprintf(log_msg, "execute command: %d", list->id);
   nh.loginfo(log_msg);
   
   if(group == 0){ // arm_group
    
    dtostrf(list->a, 6, 4, num);
    sprintf(log_msg, "Muneca V: %s", num);
    nh.loginfo(log_msg);
    moveMunecaV(list->a);
    
    dtostrf(list->b, 6, 4, num);
    sprintf(log_msg, "Muneca H: %s", num);
    nh.loginfo(log_msg);
    moveMunecaH(list->b);
    
    dtostrf(list->c, 6, 4, num);
    sprintf(log_msg, "Torso: %s", num);
    nh.loginfo(log_msg);
    moveTorso(list->c);
    
    dtostrf(list->d, 6, 4, num);
    sprintf(log_msg, "Elevador: %s", num);
    nh.loginfo(log_msg);
    
    dtostrf(list->e, 6, 4, num);
    sprintf(log_msg, "Codo: %s", num);
    nh.loginfo(log_msg);
    moveCodo(list->e);
  
  }else if(group ==1){ // gripper
    
    dtostrf(list->a, 6, 4, num);
    sprintf(log_msg, "Garra D: %s", num);
    nh.loginfo(log_msg);
    moveGarraD(list->a);
    
    dtostrf(list->b, 6, 4, num);
    sprintf(log_msg, "Garra I: %s", num);
    nh.loginfo(log_msg);
    moveGarraI(list->b);
    
  }else if(group == 2){ // neck
    
    dtostrf(list->a, 6, 4, num);
    sprintf(log_msg, "Neck: %s", num);
    nh.loginfo(log_msg);
    
  } 
}

void deleteList(){
  listSize=0;
  node* current = head;
  node* next = nullptr;

  while(current != nullptr){
    next = current->next;
    delete (current);
    current = next;
  }
}

void rosList(const sensor_msgs::JointState& cmd_msg){
  callback_counter++;
  sprintf(log_msg,"Callback Counter: %d", callback_counter);
  nh.loginfo(log_msg);
  
  int arrSize = cmd_msg.position_length;
    
  if(arrSize == 5){ // arm_group

    pushList(0, cmd_msg.position[0], cmd_msg.position[1], cmd_msg.position[2], cmd_msg.position[3], cmd_msg.position[4], callback_counter);
    
  }else if(arrSize ==2){ // gripper
    
     pushList(1, cmd_msg.position[0], cmd_msg.position[1], 0,0,0, callback_counter);

    
  }else if(arrSize == 1){ // neck
    
     pushList(1, cmd_msg.position[0], 0, 0,0,0, callback_counter);
  } 
  last_callback = millis();
}
