class Speaker{
  public:
    Speaker();
    void speak(int);
};

Speaker::Speaker(){
}

void Speaker::speak(int command){
  if(command == 0){
    cout << "STOP" << endl;
  }
  if(command == 1){
    cout << "FORWARD" << endl;
  }
  if(command == 2){
    cout << "LEFT" << endl;
  }
  if(command == 3){
    cout << "RIGHT" << endl;
  }
}
