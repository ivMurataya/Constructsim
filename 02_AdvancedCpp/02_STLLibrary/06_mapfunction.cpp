#include <iostream>
#include <map>
using namespace std;

int main()
{

    // empty map container
    map<int, char> function_test;

    // insert elements in random order
    function_test.insert(pair<int, char>(1, 'M'));
    function_test.insert(pair<int, char>(2, 'O'));
    function_test.insert(pair<int, char>(3, 'V'));
    function_test.insert(pair<int, char>(4, 'E'));

    // printing map 
    map<int, char>::iterator itr;
    cout << "\nThe map function_test is : \n";
    cout << "\tKEY\tELEMENT\n";
    for (itr = function_test.begin(); itr != function_test.end(); ++itr) {
        cout << '\t' << itr->first << '\t' << itr->second
            << '\n';
    }
    cout << endl;

    // assigning the elements from function_test to copy_function
    map<int, char> copy_function(function_test.begin(), function_test.end());

    // print all elements of the map copy_function
    cout << "\nThe map copy_function after"
        << " assign from function_test is : \n";
    cout << "\tKEY\tELEMENT\n";
    for (itr = copy_function.begin(); itr != copy_function.end(); ++itr) {
        cout << '\t' << itr->first << '\t' << itr->second
            << '\n';
    }
    cout << endl;

    // remove all elements up to
    // element with key=3 in copy_function
    cout << "\ncopy_function after removal of"
            " elements less than key=2 : \n";
    cout << "\tKEY\tELEMENT\n";
    copy_function.erase(copy_function.begin(), copy_function.find(2));
    for (itr = copy_function.begin(); itr != copy_function.end(); ++itr) {
        cout << '\t' << itr->first << '\t' << itr->second
            << '\n';
    }

    cout << endl;

    // lower bound and upper bound for map function_test key = 3
    cout << "function_test.lower_bound(3) : "
        << "\tKEY = ";
    cout << function_test.lower_bound(3)->first << '\t';
    cout << "\tELEMENT = " << function_test.lower_bound(3)->second
        << endl;
    cout << "function_test.upper_bound(3) : "
        << "\tKEY = ";
    cout << function_test.upper_bound(3)->first << '\t';
    cout << "\tELEMENT = " << function_test.upper_bound(3)->second
        << endl;

    return 0;
}
/*
The map function_test is :
        KEY     ELEMENT
        1       M
        2       O
        3       V
        4       E


The map copy_function after assign from function_test is :
        KEY     ELEMENT
        1       M
        2       O
        3       V
        4       E


copy_function after removal of elements less than key=2 :
        KEY     ELEMENT
        2       O
        3       V
        4       E

function_test.lower_bound(3) :  KEY = 3         ELEMENT = V
function_test.upper_bound(3) :  KEY = 4         ELEMENT = E
*/
