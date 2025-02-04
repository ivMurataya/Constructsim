#include <iostream>

using namespace std;

class Shape {
   public:
      //definition of a pure virtual function
      virtual int Area() = 0; 
      // Function to set dimension
      void setDimension(int w, int h) {
         width = w;
      height = h;
      }
     protected:
      int width;
      int height;
};

// A rectangle is a shape; it inherits shape.
class Rectangle: public Shape {
   public:
      int Area() { 
         return (width * height); 
      }
};
// A triangle is a shape; it inherits shape.
class Triangle: public Shape {
   public:
      // Observe the different return mathematical formula as per the need
      // We have use the same function of the base class
      int Area() { 
         return (width * height)/2; 
      }
};

int main() {
  Rectangle R;
  Triangle T;

  R.setDimension(2,4);
  T.setDimension(8,4);

  cout << "The area of the rectangle is: " << R.Area() << endl;
  cout << "The area of the triangle is: " << T.Area() << endl;
}
/*
The area of the rectangle is: 8
The area of the triangle is: 16*/
