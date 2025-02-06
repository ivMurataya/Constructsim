#include <iostream>
 
using namespace std;

class Icetray {
   public:
      static int refillCount;
           
      // Constructor definition
      Icetray(int crystals=2) {
         cout <<"Constructor called." << endl;
         cout <<  crystals << " crystals are ready"<<endl;	 
         // Increase every time object is created
         refillCount++;
      }
};

// Initialize static member of class
int Icetray::refillCount = 0;

int main(void) {
   // Declare object 1
   Icetray Orange(10);
   // Declare object 2
   Icetray Lemon(20);
   
   // Print total number of objects.
   cout << "Total refill counts: " << Icetray::refillCount << endl;

   return 0;
}
/*

clang++ demo_361.cpp -o demo_361

Constructor called.
10 crystals are ready
Constructor called.
20 crystals are ready
Total refill counts: 2
*/
