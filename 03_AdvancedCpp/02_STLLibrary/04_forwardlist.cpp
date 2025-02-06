#include<iostream> 
#include<forward_list>  
using namespace std; 
  
int main() 
{ 
    // Declaring forward list 
    forward_list<int> flist1 ={10,22,31}; 
    forward_list<int> flist2; 
    flist2.assign(5, 10); 
  
    // Displaying forward lists 
    cout << "The elements of first forward list are : "; 
    for (int i: flist1) 
        cout << i << " "; 
    cout << endl; 
      
    cout << "The elements of second forward list are : "; 
    for (int&b : flist2) 
        cout << b << " "; 
    cout << endl; 
  
    return 0; 
} 
/*
The elements of first forward list are : 10 22 31
The elements of second forward list are : 10 10 10 10 10
*/
