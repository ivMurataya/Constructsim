#include "unit6_exercises/magic_printer.h"
#include <string>

int main(int argc, char **argv) {

  ros::init(argc, argv, "magic_print_main_node");

  ros::NodeHandle _n("magic_print_ns");

  MagicPrinter magic_printer_object(_n);

  int value_integer = 42;
  magic_printer_object.PrintInteger(value_integer);

  string vallue_string = "Adventure Time!";
  magic_printer_object.PrintString(vallue_string);

  magic_printer_object.PrintGeneric<int>(value_integer);
  magic_printer_object.PrintGeneric<string>(vallue_string);

  

  return 0;
}
