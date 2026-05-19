#include "../include/ballistics.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>

int parseBallisticInput(const std::string& pathToFile, BallisticInput& out_bi)
{
  std::ifstream input(pathToFile);

  if (!input.is_open()) {
    std::cerr << "Input file was not found or unable to open, path: " << pathToFile << '\n';
    return 1;
  }

  input >> out_bi.xd >> out_bi.yd >> out_bi.zd >> out_bi.targetX >> out_bi.targetY >> out_bi.v0 >> out_bi.accelerationPath >>
    out_bi.ammo_name;
  input.close();

  return 0;
}

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: tests_code_quality <input_path>\n";
    return 1;
  }

  BallisticInput ballisticInput{};
  const int parseResult = parseBallisticInput(argv[1], ballisticInput);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  if (parseResult != 0) {
    return 1;
  }

  FirePoint firePoint;
  calculateFirePoint(ballisticInput, firePoint);

  if (!firePoint.success) {
    return 1;
  }

  std::ofstream output("output.txt");

  if (firePoint.hasManeuver) {
    output << firePoint.maneuverX << ' ' << firePoint.maneuverY << ' ';
  }

  output << firePoint.fireX << ' ' << firePoint.fireY << '\n';
  output.close();

  return 0;
}