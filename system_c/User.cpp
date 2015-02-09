#include "User.h"

void User::reset(void){
  // Reset the x,y,z,yaw coordinates and re-orient,
	x = 0;
	y = 0;
	z = 0;
	yaw = 0;
  // get pose and slam again just in case
}

void User::getPose(void){
  // Get the yaw, at this point we only care about 2D rotation in top down map
    yaw.write(myYaw);
	x.write(myX);
	y.write(myY);
	z.write(myZ);
}

void User::move(void){
  // Tell the user to move, and update the world, this is only forward for now
  // So I'm thinking about this like the spaceship in Futurama
  // You don't move the user through space you move the space around the user
	// Just moving along the x position for now
	if(forward.read() == 1){
		myX++;
		x.out(myX);
	}
}

void User::turn(void){
  // Change the orientation of the user(at an intersection for example)
  // Then tell them to move forward
	// 0 degrees is facing directly down the X line or down the hallway we want to walk down
	// 180 would be facing back down the hall way
	// 90 and 270 would be facing N or S if we view the hallway as heading E (left to right) when going forward
	if(left.read() = 1){
		myYaw = myYaw - 90;
		if(myYaw < 0){
			myYaw = 360 + myYaw;
		}
	}else if(right.read() = 1){
		myYaw = (myYaw + 90) % 360;
		if(myYaw > 359){
			myYaw = 360 - myYaw;
		}
	}
}

void User::tracking(void){
	// When a change in place occurrs the world will write the new x,y,z
	// coordinated to the internal variables of the User to store the position
	myX = x.read();
	myY = y.read();
	myZ = z.read();
}