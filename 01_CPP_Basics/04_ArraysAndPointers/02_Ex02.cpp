#include <iostream>

using namespace std;

void squared_array(int myarray[]) {
    
    // Declare pointer
    int *mypointer;
    // Assign array to pointer
    mypointer = myarray;
    // For loop
    for (int i=0; i<6; i++)
    {
        // Multiply each value by themselves
        *mypointer = (*mypointer)*(*mypointer);
        // Print value
        cout << *mypointer << endl;
        // Increment pointer
        mypointer++;
    }
}

int main() {
    int myarray[] = {4,8,15,16,23,42};
    squared_array(myarray);
    return 0;
}
