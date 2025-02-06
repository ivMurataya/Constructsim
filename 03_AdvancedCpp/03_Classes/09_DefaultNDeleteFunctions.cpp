#include <iostream>

class Icetray {
public:
    // a user-defined
    // parameterized constructor
    Icetray(int x){
        std::cout << "This is a parameterized constructor" << std::endl;
    }
    
    /* Using the default specifier to instruct
     the compiler to create the default
    implementation of the constructor.*/
    Icetray() = default;
    // make copy constructor uncallable
    Icetray(const Icetray&)= delete;
};

int main()
{
    // executes using default constructor
    Icetray apple_juice, lemon_juice;
    
    // uses parameterized constructor
    Icetray mango_juice(1);

    // error, the usage of the 
    // copy constructor is disabled
    lemon_juice = Icetray(apple_juice);
    return 0;
}
