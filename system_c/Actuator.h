#ifndef ACTUATOR_H
#define ACTUATOR_H

class Actuator{

  int steps = 0;

  public:
    Actuator(int);
    int go_Forward();
    void stop();

};

// Constructor
Actuator::Actuator(int stride){
}

// Again, defining methods here no Actuator.c
int Actuator::go_Forward(){
  steps ++;
  return steps;
}

void Actuator::stop(){
  steps = 0;
}

void Actuator::go_Left(){
  //Todo
}

void Actuator::go_Right(){
  //Todo
}
