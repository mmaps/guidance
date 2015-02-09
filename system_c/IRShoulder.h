#ifndef IRSHOULDER_H
#define IRSHOULDER_H

#include <systemc.h>

SC_MODULE(IRShoulder){

  //Input from processor: Triggers the sensor to "get a reading"
  sc_in<sc_bit> trigger;
  //Output to processor: writes out the distance gotten from the world module
  sc_out<sc_uint<10>> distance;
  //Input from world: is the distance to the walls
  sc_in<sc_uint<16>> leftWall;
  sc_in<sc_uint<16>> rightWall;
  sc_in<sc_uint<16>> aheadWall;
  sc_in<sc_uint<16>> backWall;
  //Output to the world: says give me your walls!
  sc_out<sc_bit> triggerWorldUpdate;

  //Processes
  void read(void);

  //Construktor
  SC_CTOR(IRShoulder) {
      SC_METHOD(read);
      	sensitive << trigger.pos();
  }
}
