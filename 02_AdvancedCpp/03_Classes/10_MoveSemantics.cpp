#include <iostream>
#include <vector>
using namespace std;

class Icetray{
  int *ptr;
public:
  Icetray(){
    // Default constructor
    cout << "Calling Default constructor\n";
    ptr = new int ;
  }

  Icetray( const Icetray & obj){
    // Copy Constructor
    // copy of object is created
    this->ptr = new int;
    // Deep copying
    cout << "Calling Copy constructor\n";
  }
  //In absence of the move constructor, copy constructor will be called
  Icetray ( Icetray && obj){
    // Move constructor
    // It will simply shift the resources,
    // without creating a copy.
    cout << "Calling Move constructor\n";
    this->ptr = obj.ptr;
    obj.ptr = NULL;
  }

  ~Icetray(){
    // Destructor
    cout << "Calling Destructor\n";
    delete ptr;
  }

};

int main() {

  vector <Icetray> vec;
  
  // temporary object is pushed back to the vector
  vec.push_back(Icetray());
  // move constructor can be explicitly called, with the move() function, for already created objects
  return 0;
}
/*
Calling Default constructor
Calling Move constructor
Calling Destructor
Calling Destructor 
*/
