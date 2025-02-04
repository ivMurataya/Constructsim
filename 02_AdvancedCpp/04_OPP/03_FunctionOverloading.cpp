#include <iostream>
#include <string>


// method 1
void mobile(int var)
{
    std::cout << "Charge available " << var << std::endl;
}

// method 2
void mobile(std::string s)
{
    std::cout << "Battery Status: "<< s << std::endl;
}


int main()
{
    int a = 96;
    std::string status ="Charging";

    mobile(a);
    mobile(status);

    return 0;
}
/*
Charge available 96
Battery Status: Charging*/
