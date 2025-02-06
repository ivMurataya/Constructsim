#include<iostream>
using namespace std;

class GrandDad{
    public:
        void Dfunc(){
        cout<<"GrandDad class is called"<<endl;
        }
};

class GrandMom{
    public:
        void Mfunc(){
        cout<<"GrandMom class is called"<<endl;
        }
};

class Father : public GrandDad{
    public:
        void Ffunc(){ 
            cout<<"Father class is called"<<endl;
        }
};
    
class Uncle  : public GrandDad, public GrandMom{
    public:
};
    
class You : public Father{
    public:
};

int main(){

Father Bob;
cout << "Bob is example of Single Inheritance"<<endl;
Bob.Dfunc();
Uncle Mario;
cout << "Mario is an example of Multiple Inheritance"<<endl;
Mario.Dfunc();
Mario.Mfunc();
You Tom;
cout<< "Tom is an example of MultiLevel Inheritance"<<endl;
Tom.Dfunc();
Tom.Ffunc();
return 0;
}
/*
Bob is example of Single Inheritance
GrandDad class is called
Mario is an example of Multiple Inheritance
GrandDad class is called
GrandMom class is called
Tom is an example of MultiLevel Inheritance
GrandDad class is called
Father class is called
*/
