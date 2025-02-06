#include<iostream>

class IceTray
{
 public:
    std::string fish_shape;
    std::string star_shape;

    void status(){
      std::cout << "Crystal which is in fish shape is "<< fish_shape << " and the one in" << uwu << " star shape is " << star_shape << std::endl;
  
  }

  private:
    std::string uwu = "uwu";
};

int main(){
  
  // declare an object of class IceTray
  IceTray orange_juice;

  // assign a value to member variables
  orange_juice.fish_shape = "frozen";
  orange_juice.star_shape = "not frozen";

  // call a class method or member function
  orange_juice.status();
  return 0;
}
