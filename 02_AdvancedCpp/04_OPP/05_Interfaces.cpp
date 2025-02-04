#include<iostream>

// implement interface using abstract class
// interface contains only pure virtual member function
class IPower
{
  public:
  virtual void Square()=0;
  virtual void Cube()=0;
  ~IPower()=default;
};

class SomeData : public IPower
{
public:
  SomeData(int x, int y, int z)
  {
    X = x;
    Y = y;
    Z = z;
  }

void Square()
{
  std::cout << "Square of the provided numbers are:\n";
  std::cout << X*X << std::endl;
  std::cout << Y*Y << std::endl;
  std::cout << Z*Z << std::endl;
}
void Cube()
{
  std::cout << "Cube of the provided numbers are:\n";
  std::cout << X*X*X << std::endl;
  std::cout << Y*Y*Y << std::endl;
  std::cout << Z*Z*Z << std::endl;
  
}
private:
  int X;
  int Y;
  int Z;
};

int main(){
    SomeData numbers(4,2,7);
    numbers.Square();
    numbers.Cube();
    return 0;
}

/*
Square of the provided numbers are:
16
4
49
Cube of the provided numbers are:
64
8
343
*/
