Motor::Motor() {}


Motor::Motor(byte b, byte c) {
  this->defineOutput();
  this->m1 = b;
  this->m2 = c;
  this->state=3;
  this->changePWM(200);
}
void Motor::defineOutput() {
  pinMode(this->m1, OUTPUT);
  pinMode(this->m2, OUTPUT);
}
void Motor::changePWM(byte p){
  this->pwm=p;
  switch(state){
    case 1:
      this->Forward();
    break;
    case 2:
      this->Backward();
    break;
    case 3:
      this->Stop();
    break;
  }
}

void Motor::Stop() {
  analogWrite(this->m1, 0);
  analogWrite(this->m2, 0);
  this->state=3;
}
void Motor::Forward() {
  analogWrite(this->m1, this->pwm);
  analogWrite(this->m2, 0);
  this->state=1;
}
void Motor::Backward() {
  analogWrite(this->m1, 0);
  analogWrite(this->m2, this->pwm);
  this->state=2;
}
