#include<iostream>
#include<vector>

struct IceTray
{
  std::string name;
  int crystal_numbers;
  std::vector<std::string> shapes;
  float time_;
 
};

void print_details(IceTray tray){
  
  std::cout<< "Details for '"<< tray.name << "' tray are:"<< std::endl;
  std::cout << "Crystal count : "<< tray.crystal_numbers<< std::endl;
  std::cout << "Time required : "<< tray.time_<< std::endl;
  std::cout << "Shapes available are:" << std::endl;
  for (auto i = tray.shapes.begin(); i != tray.shapes.end(); ++i)
        std::cout << *i << " " << std::endl;

}


int main(){
  IceTray green_tray;
  green_tray.name = "Green";
  green_tray.crystal_numbers = 10;
  green_tray.shapes.push_back("fish");
  green_tray.shapes.push_back("crab");
  green_tray.shapes.push_back("star");
  green_tray.time_ = 20.2;
  print_details(green_tray);

  return 0;
}
/*
Details for 'Green' tray are:
Crystal count : 10
Time required : 20.2
Shapes available are:
fish
crab
star*/
