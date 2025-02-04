#include <iostream>

void func(int* u){
    std::cout << "Output of constant cast " << *u << std::endl;
}

class Base{
    virtual void print(){}
};
class derived : public Base{
};
int main()
{
    // using static cast operator
    float f = 3.5;
    int b = static_cast<int>(f);
    std::cout << "Output of static cast " << b << std::endl;

    // using constant cast operator
    const int num1 = 40;
    const int *num2 = &num1;
    
    // typecasting to pass a pointer to constant data type variable
    int *v = const_cast <int*>(num2);
    func(v);

    // using dynamic cast operator
    Base *ptr = new derived;
    
    // converting pointer to parent into pointer to child class
    derived *pd = dynamic_cast<derived*>(ptr);
    
    if(pd!=NULL){
        std::cout << "Dynamic casting is done successfully" << std::endl;
    }
    else{
    std::cout<< "Dynamic casting has failed" << std::endl;
    }

    // using reinterpret cast operator
    // declare a pointer variable
    int *pt = new int (65);
    
    // converting int pointer type to char pointer type
    char *ch = reinterpret_cast <char*>(pt);
    std::cout << "The value of pt is "<< pt << ", value of pt* is "<< *pt << std::endl;
    std::cout << "The value of ch is "<< ch << ", value of ch* is "<< *ch << std::endl;

    return 0;
}
/*
Output of static cast 3
Output of constant cast 40
Dynamic casting is done successfully
The value of pt is 0x10552e0, value of pt* is 65
The value of ch is A, value of ch* is A
*/
