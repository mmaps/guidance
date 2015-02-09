#ifndef USER_H
#define USER_H

#include <systemc.h>

SC_MODULE(User){
  //Ports
  // The outputs will go to the world module
  // The world module takes them and moves the pointer or what-have-you
  // through the array/graph/what-have-you
  sc_inout <sc_int<16>> x,y,z;
  sc_out <sc_int<16>> yaw;

  //Output to the world module: triggers an update in location
  sc_out <sc_bit> move;

  sc_out <sc_bit> reset;
  //Input from the processor: instructs the user to move
  sc_in <sc_bit> left;
  sc_in <sc_bit> right;
  sc_in <sc_bit> forward;
  // I don't think we need to move back
  //sc_in <sc_bit> back;
  sc_in <sc_bit> update;

  //Create Channels
  //TODO

  //Local Variables
  int myYaw = 0;
  int myX = 0;
  int myY = 0;
  int myZ = 0;

  //Process Declarations
  void move();
  void turn();
  void reset();
  void getPose();
  void tracking();

  //Constructor
  SC_CTOR(User){

    SC_METHOD(tracking);
	  sensitivity << x;
	  sensitivity << y;
	  sensitivity << z;

    SC_METHOD(reset);
      sensitivity << reset;
    
    SC_METHOD(move);
      sensitivity << forward;

    SC_METHOD(turn);
      sensitivity << left;
      sensitivity << right;

    SC_METHOD(getPose);
	  sensitivity << update;
  }
};

#endif

