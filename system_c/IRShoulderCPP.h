#ifndef IRSHOULDER_H
#define IRSHOULDER_H

#include "WorldCPP.h"

class IRShoulder {

  int theta;

  public:
    IRShoulder(int);
    int get_Distance(int);

};

IRShoulder::IRShoulder(int orientation){
  theta = orientation;
}

// Small class, defined the function here so we don't
// need the IRShoulder.c

int IRShoulder::get_Distance(int userOrientation){

  int distance = 0;
  if(theta == 270){
    distance = world.get_Wall(userOrientation)[3];
  }
  if(theta == 90){
    distance = world.get_Wall(userOrientation)[1];
  }
  
}
