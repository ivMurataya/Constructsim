#include <iostream>
using namespace std;

class Complex {
    
   public:
    float real;
    float imag;

    // Constructor to initialize real and imag to 0
    Complex() : real(0), imag(0) {}

    
    // Overload the + operator
    Complex operator + (const Complex& obj) {
        Complex temp;
        temp.real = real + obj.real;
        temp.imag = imag + obj.imag;
        return temp;
    }

    void output() {
        if (imag < 0)
            cout << "Output Complex number: " << real << imag << "i\n";
        else
            cout << "Output Complex number: " << real << "+" << imag << "i\n";
    }
};

int main() {
    Complex complex1, complex2, result;
    complex1.real= 3;
    complex1.imag= 4;

    complex2.real = 4;
    complex2.imag= 1;

   // complex1 calls the operator function
   // complex2 is passed as an argument to the function
    result = complex1 + complex2;
    result.output();

    return 0;
}
// Output Complex number: 7+5i
