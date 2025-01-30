#include<iostream>
#include<set>
using namespace std;
int main()
{
    multiset < char > s;
  
    // inserting elements in random order .
    s.insert( 'R' ) ;
    s.insert( 'O' ) ;
    s.insert( 'B' ) ;
    s.insert( 'O' ) ;
    s.insert( 'T' ) ;
     
    // printing set s
    //initialising the iterator, iterating to the beginning of the set.
 
    set<char >::iterator it ;
    cout << "The element of set s are : \n";
    for (it = s.begin() ; it != s.end() ; it++ ) 
    {
        cout << *it<<" ";
    }
    cout << endl;
    cout<< "The size of set :\n " << s.size() <<endl ;
    return 0 ;
}
/*
The element of set s are :
B O O R T
The size of set :
 5
 */
