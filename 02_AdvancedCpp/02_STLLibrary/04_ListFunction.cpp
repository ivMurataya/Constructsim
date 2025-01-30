#include <iostream>
#include <list>

using namespace std;
void showlist(list<int> k){
    for (int i : k){
        cout << i << " ";
    }
    cout<< "\n";
}
int main()
{
    list<int> l1{1,2,3,4,8};
    showlist(l1);
    list<int> l2{5,7,9};
    l1.swap(l2);
    showlist(l1);
    l1.reverse();
    showlist(l1);
    l1.sort();
    showlist(l1);
    l1.merge(l2);
    showlist(l1);
    return 0;
}
