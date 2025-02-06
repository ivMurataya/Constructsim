#include <iostream>
#include <map>

using namespace std;

int main ()
{

    map<string,string> food; 
    /*  creates a map with keys of type character and 
      values of type integer */
    
    food["Tom"]="Hot chocolate cake";    // inserts key = "Tom" with value = Hot chocolate cake
    food["Jerry"]="Cheese Cake";      
    food["Aunt"]="Crispy clothes";      
    cout<< food.at("Tom")<<", "<<food["Jerry"]<<endl;
    
    map<string, string> copy_food (food);
    /* creates map copy_food which is a copy of map food */
    return 0;
}
/*
Hot chocolate cake, Cheese Cake
*/
