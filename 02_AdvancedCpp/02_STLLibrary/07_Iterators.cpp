#include<iostream>
#include<list>
using namespace std;
int main()
{
    //Don't forget to include the required header file
    list<int> v{1,2,3,4,4,1,6,7} ;
    list<int>::iterator i;
    
    for(i = v.begin(); i!= v.end(); i++){
    cout << *i <<"  ";
    // to get the value of the element pointed by iterator we use *
    }
    cout<<"\n";

return 0;
}
/*
1  2  3  4  4  1  6  7
*/
