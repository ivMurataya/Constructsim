#include <iostream>
#include <vector>
using namespace std;

int main() {
  // Observe different ways to declare a vector

  // method 1
  vector<int> vector1 = {1, 2, 3, 4, 5};

  // uniform initialization - method 2
  vector<int> vector2{6, 7, 8, 9, 10};

  // method 3
  vector<int> vector3(5, 12);

  cout << "vector1 = ";

  //ranged loop
  for (const int& i : vector1) {
    cout << i << "  ";
  }

  cout << "\nvector2 = ";

  // ranged loop
  for (const int& i : vector2) {
    cout << i << "  ";
  }

  cout << "\nvector3 = ";

  // ranged loop
  for (int i : vector3) {
    cout << i << "  ";
  }

  //Add, access and remove a number from vector

  //add 59 to vector 1
  vector1.push_back(59);
  cout << "\nEdited vector1 = ";
  for (const int& i : vector1) {
    cout << i << "  ";
  }

  //Access 2nd element(index 1) of vector 2
  cout << "\n2nd element of vector 2: "<<vector2.at(1)<< endl;

  //Remove last element of vector 3
  vector3.pop_back();
  cout << "Edited vector3 = ";
  for (int i : vector3) {
    cout << i << "  ";
  }


  return 0;
}
