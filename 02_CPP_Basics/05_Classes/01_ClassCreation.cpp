#include <iostream>
#include <string>
using namespace std;

class Hobbit {
    public:
       string name;
       float height; //in meters
       int age;
    
       void get_name() {
           cout << "This hobbit's name is " << name << endl;
       }
       void convert_height_to_cm();
};

void Hobbit::convert_height_to_cm() {
    height = height*100;
    cout << "Height of the hobbit is " << height << " centimeters" << endl;
}

int main() {
    
    Hobbit first_hobbit;
    
    first_hobbit.name = "Frodo";
    first_hobbit.age = 28;
    first_hobbit.height = 1.80
    first_hobbit.get_name();

        first_hobbit.convert_height_to_cm();


    
}
