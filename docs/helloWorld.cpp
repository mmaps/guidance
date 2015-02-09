#include "systemc.h"

//g++ -I/usr/local/systemc-2.3/include -L/usr/local/systemc-2.3/lib-linux64/ -Wl,-rpath=/usr/local/systemc-2.3/lib-linux64 helloWorld.cpp -lsystemc


/* And Kyra can change the files....*/

SC_MODULE (hello_world){
  SC_CTOR(hello_world){
  }

  void say_hello(){
    cout << "Hello World.\n";
  }
};

int sc_main(int argc, char* argv[]){
  hello_world hello("HELLO");
  hello.say_hello();
  return(0);
}
