#include<iostream>

class IceTray
{
  public:
    
    // default constructor
    IceTray(){
    // initializing the data members of the class
    fish_shape = "frozen";
    star_shape = "not frozen";
    }
    
    // parameterized constructor
    IceTray(std::string fish_shape_state, std::string star_shape_state){
    fish_shape = fish_shape_state;
    star_shape = star_shape_state;
    }
    
    // destructor
    ~IceTray(){
    std::cout << "Destructor is called" << std::endl;
  }
   
    std::string fish_shape;
    std::string star_shape;

    void status(){
      std::cout << "Crystal which is in fish shape is "<< fish_shape << " and the one in star shape is " << star_shape << std::endl;
  
  }
};

int main(){
  
  // initialize an object of class IceTray using default constructor
  IceTray orange_juice;
    
  // initialize an object of class IceTray using parametrized constructor
  IceTray lemon_juice("frozen", "frozen");

  // accessing a class method or member function
  orange_juice.status();
  lemon_juice.status();
    
  return 0;
}
/*
  clang++ demo_35.cpp -o demo_35
./demo_35

  Crystal which is in fish shape is frozen and the one in star shape is not frozen
Crystal which is in fish shape is frozen and the one in star shape is frozen
Destructor is called
Destructor is called

*/
