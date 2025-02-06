#include<iostream>

class Icetray {
   int crystals;
   public:
   Icetray(int x = 0) {
      crystals = x;
   }
   // non-constant function
   int getValue1(){
   crystals++;
   return crystals;
   }

   // constant member function
   int getValue2() const {
      return crystals;
   }
};

int main() {
    
   // constant object, can only call constant function
   const Icetray apple_juice(5);
   std::cout << "getValue2() using const object apple_juice : " << apple_juice.getValue2() << std::endl;
    
   // non-constant object
   Icetray lemon_juice(8);
   std::cout << "getValue1() using non-const object lemon_juice : " << lemon_juice.getValue1() << std::endl;
   std::cout << "getValue2() using non-const object lemon_juice : " << lemon_juice.getValue2() << std::endl;
   return 0;
}



/*
getValue2() using const object apple_juice : 5
getValue1() using non-const object lemon_juice : 9
getValue2() using non-const object lemon_juice : 9
*/
