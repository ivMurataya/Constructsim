#include <iostream>
using namespace std;

class Icetray {
private:
  int crystals;
  string shape;
public:
  void setMyValues(int crystals, string shape){
    //let the compiler know that values are to be assigned
    //to the same object calling the function
    this->crystals =crystals;
    this->shape=shape;
  }
  void displayMyValues(){
    cout<<crystals<<" , ";
    cout<<shape<<endl;
  }
};
int main(){
  Icetray Lemon_juice;
  Lemon_juice.setMyValues(10, "Tree");
  Lemon_juice.displayMyValues();
  return 0;
}
/*
10 , Tree
*/
