#include<iostream>

class Icetray{
  public:
  int crystals;
    
  // overload operator =
  Icetray& operator= (Icetray&& source) {
    this->crystals = source.crystals;
    source.crystals = 0;
    return *this;
    }
};

int main(){
  
  // move the values from orange_tray to another existing object lemon_tray 
  Icetray orange_tray;
  orange_tray.crystals = 5;
  Icetray lemon_tray;
    
  // call Icetray& operator= (Icetray&& source)
  lemon_tray = std::move(orange_tray); 
    
  std::cout << "Number of crystals in orange juice tray: "<< orange_tray.crystals << std::endl;
  std::cout << "Number of crystals in lemon juice tray: " << lemon_tray.crystals << std::endl; 
    
  return 0;
}
/*
Number of crystals in orange juice tray: 0
Number of crystals in lemon juice tray: 5
*/
