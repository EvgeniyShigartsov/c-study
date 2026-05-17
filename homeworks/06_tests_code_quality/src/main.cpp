#include "../include/ballisticts.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: tests_code_quality <input_path>\n";
    return 1;
  }

  FirePoint firePoint;
  const int result = calculateFirePoint(argv[1], firePoint);

  if (result != 0) {
    return 1;
  }

  std::ofstream output("output.txt");

  if (firePoint.hasManeuver) {
    output << firePoint.maneuverX << ' ' << firePoint.maneuverY << ' ';
  }

  output << firePoint.fireX << ' ' << firePoint.fireY << std::endl;
  output.close();

  return 0;
}