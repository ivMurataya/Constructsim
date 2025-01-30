#include <iostream>
#include <list>

using namespace std;

int main()
{
    list<int> l{1,2,3};
    l.push_back(5);
    l.pop_front();
    
    for(int i:l){
        cout << i <<" ";
    }
    cout << "\n";
}
