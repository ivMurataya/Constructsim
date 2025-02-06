#include<iostream>
#include<list>
using namespace std;

int main(){
    
    list<string> laser{"range_max","range_min","angle_max","angle_min"};
   
    //iterator laser_front starts to point the first element of the list
    auto laser_front = laser.begin();
    //Jump 2 more times as you are already on the first element
    advance(laser_front, 2);
    // * is used to tell the value of what is being pointed by the iterator
    cout << *laser_front << '\n';

    //iterator laser_front starts to point the past-the-end element of the list
    laser_front = laser.end();
    //Jump back to the third element from last
    laser_front = next(laser_front, -3);
    cout << *laser_front<< '\n';


    return 0;
}
