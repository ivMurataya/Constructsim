#include <iostream>

int main(){
    // variable declaration 
    int my_variable = 2;
    // pointer variable declaration
    int * my_pointer;
    // store memoery address of variable my_variable
    my_pointer = &my_variable;    

    // following lines will generate error
    // as values can not be assigned to pointers
    // my_pointer = 2; 
    // my_pointer = my_variable;
    
    std::cout << "my_variable = " << my_variable << " (value of my_variable)" << std::endl;
    std::cout << "&my_variable = " << &my_variable << " (address of my_variable)" << std::endl;
    std::cout << "my_pointer = " << my_pointer << " (address of my_variable)" << std::endl;
    std::cout << "*my_pointer = " << *my_pointer << " (value store at address 'my_pointer')" << std::endl;
    

    return 0;
}
/*
my_variable = 2 (value of my_variable)
&my_variable = 0x7ffcb7307158 (address of my_variable)
my_pointer = 0x7ffcb7307158 (address of my_variable)
*my_pointer = 2 (value store at address 'my_pointer')*/
