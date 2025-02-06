#include <iostream>
using namespace std;

class Icetray  {
private:
  int crystals;
  string shape;
public:
  
  Icetray &setCrystals(int crystals){
    this->crystals = crystals;
    //return the reference of current object  
    return *this;
  }
  Icetray &setShape(string shape){
    //chaining is sequential and the changes made to the object’s 
    //data members here crystals retains for further chaining calls
    this->crystals--;
    //thus we get 9 as output
    this->shape =shape;
    return *this;
  }
  void displayMyValues(){
    cout<< crystals <<" , ";
    cout<< shape <<endl;
  }
};
int main(){
  Icetray Lemon_juice;
  //Chaining calls
  //call all the functions for the current object in one go
  Lemon_juice.setCrystals(10).setShape("Tree");
  Lemon_juice.displayMyValues();
  return 0;
}
/*
9 , Tree
*/
