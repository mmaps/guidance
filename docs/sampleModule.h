#include "systemc.h"

//SC_MODULE(module_name){
  //Ports declaration
  //Signals declaration
  //Module constructor: SC_CTOR
  //Process constructors and sensitivity list
  //  SC_METHOD
  //Sub-modules creation and port mapping
  //Signals initilization
//}

SC_MODULE(FullAdder){

  sc_in<sc_uint<16>> A;
  sc_in<sc_uint<16>> B;
  sc_out<sc_uint<17>> result;

  void dolt(void);

  SC_CTOR(FullAdder){

    SC_METHOD(dolt);
      sensitive << A;
      sensitive << B;
  }
};
