#include<iostream>
#include<vector>
using namespace std;

int main()
{
    vector<int>  v{1,2,3,4,4,6,7};    // create a vector
    vector<int>::iterator i,j;  // defines an iterator i to the vector of integers
    cout<<"The vector we have : ";
    for(i= v.begin() ; i!=v.end();i++){
        cout << *i <<" ";
    }
    cout<<"\n";

    i = v.begin();
    cout << "Begin() : "<<*i << endl;
    /* i now points to the beginning of the vector v */
    
    advance(i,5);
    cout <<"Advance by 5 : "<< *i << endl;
    
    advance(i,-1);
    cout << "Move back by 1 : "<<*i << endl;
    
    i = v.begin();
    j = v.end();
    
    cout <<"Distance between begin and end : "<< distance(i,j) << endl;
    next(i,4);
    cout <<"4 elements next to i's current position(which is begin here) : " <<*next(i,4) << endl;
    advance(i,5);
    
    
    cout <<"4 elements previous to i's current position(which is at 6 here) : " <<*prev(i,4) << endl;
    return 0;
}
/*
he vector we have : 1 2 3 4 4 6 7
Begin() : 1
Advance by 5 : 6
Move back by 1 : 4
Distance between begin and end : 7
4 elements next to i's current position(which is begin here) : 4
4 elements previous to i's current position(which is at 6 here) : 2*/
