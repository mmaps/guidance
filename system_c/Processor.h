#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <systemc.h>

SC_MODULE(Processor){

  //Output to sensor: triggers a reading return
  sc_out<sc_bit> sensorTrigger;
  //Input from sensor
  sc_in<sc_uint<10>> sensorDistance;
  //Output to user
  sc_out<sc_bit> moveLeft, moveRight, moveForward;

}
