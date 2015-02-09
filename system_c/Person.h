#ifndef PERSON_H 
#define PERSON_H

#include <systemc.h>
#include "Actuator.h"
#include "Speaker.h"

SC_MODULE(Person){

  // PORTS
  // Output to the processor
  //    Position on the map/graph
  sc_out <sc_int<16>> xOut,yOut;
  //    Orientation
  sc_out <sc_int<16>> thetaOut;
  //    Sensor/Cam data
  sc_out <sc_uint<10>> IRData;
  // Not sure how to pass the camera data to the processor
  // with a systemc type. Process it in the person module?
  sc_out <camera_frame> FrameCapture;
 
  // Input from the processor
  // We could encode these into one 32-bit instruction if
  // we need to save on channels. -mikea
  // move: instructs the user to move
  //    different values for move will mean different movements
  //    a 16-bit int gives us a huge range of possible values
  sc_in <sc_uint<16>> movement;
  // get_*: bit signals from the processor
  //    signal to the Person system to return sensor data
  //    again a 16 bit encoded int gives tons of options for which sensor   
  sc_in <sc_uint<16>> get_SensorData;
  // get_Pose: proc query for pose update
  sc_in <sc_bit> get_Pose;
  // reset:
  //    F it. Start over.
  sc_in <sc_bit> reset;

  // Local Variables
  int theta = 0;
  int x = 0;
  int y = 0;
  Actuator *legs = new Actuator;
  Speaker *audio = new Speaker;

  // Process Declarations
  // move: will cause forward and turning movements
  void move();
  // reset: zero everything and re-localize
  void reset();
  // get_Pose: return the x,y,theta to the proc
  void get_Pose();

  //Constructor
  SC_CTOR(User){

    SC_METHOD(reset);
      sensitivity << reset.pos();
    
    SC_METHOD(move);
      sensitivity << movement.pos();

    SC_METHOD(get_Pose);
      sensitivity << get_Pose.pos();
  }

};

#endif

