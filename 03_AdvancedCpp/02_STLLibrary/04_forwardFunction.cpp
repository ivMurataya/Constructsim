#include<iostream> 
#include<forward_list>  
using namespace std; 

void show_fl(forward_list<int> k){
    for (int i : k){
        cout << i <<" ";
    }
    cout <<"\n";
}
int main() 
{ 
    // Initializing forward list 
    forward_list<int> flist = {10, 20, 30, 40, 50}; 
    cout << "The forward list : "; 
    show_fl(flist);
    
    /*Iterators are used to point to the containers in STL,
    because of iterators it is possible for an algorithm to manipulate 
    different types of data structures/Containers. We will learn more about iterators shortly*/
    forward_list<int>::iterator ptr; 

    flist.push_front(60);  
    cout << "The forward list after push_front operation : "; 
    show_fl(flist);
    
    flist.emplace_front(70); 
    cout << "The forward list after emplace_front operation : "; 
    show_fl(flist);
      
    flist.pop_front(); 
    cout << "The forward list after pop_front operation : "; 
    show_fl(flist);

    ptr =  flist.insert_after(flist.begin(), {1, 2, 3}); 
    cout << "The forward list after insert_after operation : ";
    show_fl(flist);

    ptr = flist.emplace_after(ptr,2); 
    cout << "The forward list after emplace_after operation : "; 
    show_fl(flist);

    ptr = flist.erase_after(ptr); 
    cout << "The forward list after erase_after operation : "; 
    show_fl(flist);

    flist.remove(40); 
    cout << "The forward list after remove operation : ";
    show_fl(flist);

    flist.remove_if([](int x){ return x>20;}); 
    cout << "The forward list after remove_if operation : ";
    show_fl(flist);
  
    return 0; 
} 
