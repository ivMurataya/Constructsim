
#include "rosbot_control/rosbot_class.h"
#include <iostream>
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "rosbot_node");
// ------------------------ CHARS and Strings ---------------------------------------
  char d[6] = "hello"; 
  
  string e = "developer";
  
  cout << d[0] << endl;        // Print function
  cout << e[4] << endl;        // Print function
  string f = ", ";
  string g = "!";
  
  cout << d+f+e+g << endl;     // Print function

  // -------------------LISTS------------------------------------------

  list<int> numbers_list({1,10,100,1000}); //list of integer values:
  list<string> vocals_list( {"a","e","i","o","u"} ); //list of string values:

  //For now, here's an example of printing a list:
  for (int val : numbers_list)             // Loop
    cout << val << "  ";                 // Print function
  
  
  for (string val : vocals_list)           // Loop
      cout << val << "  ";                 // Print function

  //Lists are very useful, as they occupy a memory space that can be modified. They have builtin functions to, for example, add a new item in the beginning of the list, or at the end of i
  numbers_list.push_front(0);             //insert in the beginning
  numbers_list.push_back(3000);           //insert in the end

  //Finally, we can also concatenate a list at the end of another, enlarging the first one and not deleting the second one, with the builtin function insert():
  list<int> new_list({5,50,500});
  numbers_list.insert(numbers_list.end(),new_list.begin(),new_list.end());
  //Then the numbers_list will be modified as 0,1,10,100,1000,3000,5,50,500

  //------------------------- DICTIONARIES-------------------------------
  /*A dictionary in C++ is called a map, and it is a container of values that are indexed by a key. This means that it stores two kinds of information: keys and values.*/
  map<string,int> girls_dictionary;
  girls_dictionary["Dolores"] = 30;
  girls_dictionary["Maeve"] = 27;
  girls_dictionary["Theresa"] = 6;
  girls_dictionary["Clementine"] = 11;
  
  for (auto item : girls_dictionary)
      cout << item.first << " appears in " << item.second << " episodes\n";
/*Finally, we can print the items in the dictionary with a loop that will give:  
      Clementine appears in 11 episodes   
      Dolores appears in 30 episodes   
      Maeve appears in 27 episodes   
      Theresa appears in 6 episodes */
//As you can see when we print it this is not a list, the dictionary itself organizes the keys alphabetically.


  return 0;
}
