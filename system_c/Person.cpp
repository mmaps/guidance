#include "Person.h"

void User::reset(void){
  // Reset the x,y,z,yaw coordinates and re-orient,
  x = 0;
  y = 0;
  theta = 0;
  // get pose and slam again just in case
}

void User::get_Pose(void){
  // Get the yaw, at this point we only care about 2D rotation in top down map
  thetaOut.write(theta);
  xOut.write(x);
  yOut.write(y);
}

void User::move(void){
  // Tell the user to movement. and update the world, this is only forward for now
  // So I'm thinking about this like the spaceship in Futurama
  // You don't movement.the user through space you movement.the space around the user
  // Just moving along the x position for now

  // STOP
  if(movement.read() == 0){
    // Tell user to stop
    audio->speak(0);
  }
  // FORWARD
  // This may need to be encapsulated in a loop
  // While !told to stop, go forward
  if(movement.read() == 1){
    audio->speak(1);
    while(movement.read() != 0){
      // Wait how long?
      wait(100);
      // go_Forward returns the number of steps taken
      // add that to the current x position
      x += legs->go_Forward();
    }
  }
  // LEFT
  if(movement.read() == 2){
    audio->speak(2);
    theta = theta - 90;
    if(theta < 0){
      theta = 360 + theta;
    }
    legs->go_Left();
  }
  // RIGHT
  if(movement.read() == 3){
    audio->speak(3);
    theta = (theta + 90) % 360;
    if(theta > 359){
      theta = 360 - theta;
    }
    legs->go_Right();
  }
}
