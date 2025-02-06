#include <iostream>

using namespace std;
int main() {
  int myarray[4] = {9,99,999,9999};
  int* start = myarray;
  int* stop = myarray + 4;
  int* currentpointer = start;
  
  while (currentpointer != stop) {
      ++(*currentpointer);
      ++currentpointer;
  }
  for (int i =0 ; i <4 ; i ++){
    cout << myarray[i] << endl; 
  }
    return 0;
}

/*
10
100
1000
10000
*/
