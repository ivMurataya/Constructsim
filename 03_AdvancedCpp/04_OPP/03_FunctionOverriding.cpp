#include<iostream>
using namespace std;

class Mobile
{
public:
    virtual void Display()
    {
        cout << "\nThis is Display() method"
                " of base class Mobile"<<endl;
    }
    void Show()
    {
        cout << "\nThis is Show() method "
            "of BaseClass Mobile"<<endl;
    }
};

class VideoPlayer : public Mobile
{
public:
    // Overriding method - new working of
    // base class's display method
    void Display()
    {
        cout << "\nThis is Display() method"
            " of derived Class VideoPlayer"<<endl;
    }
};

// Driver code
int main()
{
    VideoPlayer Vp;
    Mobile &M = Vp;
    M.Display();
    Vp.Show();
}
/*

This is Display() method of derived Class VideoPlayer

This is Show() method of BaseClass Mobile
*/
