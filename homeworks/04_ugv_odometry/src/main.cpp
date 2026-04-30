#include <cmath>
#include <iostream>
#include <fstream>

const int ticks_per_revolution = 1024;
const float wheel_radius_m = 0.3f;
const float wheelbase_m = 1.0f;

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: ugv_odometry <input_path>\n";
    return 1;
  }

  std::ifstream inputFile(argv[1]);

  if (!inputFile.is_open()) {
    std::cerr << argv[1] << " file not found." << std::endl;
    return 1;
  }

  const float distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;

  long prev_timestamp_ms;
  long prev_fl_ticks, prev_fr_ticks;
  long prev_bl_ticks, prev_br_ticks;
  inputFile >> prev_timestamp_ms >> prev_fl_ticks >> prev_fr_ticks >> prev_bl_ticks >> prev_br_ticks;

  long timestamp_ms;
  long fl_ticks, fr_ticks;
  long bl_ticks, br_ticks;

  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;

  while (inputFile >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
    float delta_fl = fl_ticks - prev_fl_ticks;
    float delta_fr = fr_ticks - prev_fr_ticks;

    float delta_bl = bl_ticks - prev_bl_ticks;
    float delta_br = br_ticks - prev_br_ticks;

    float delta_left = (delta_fl + delta_bl) / 2;
    float delta_right = (delta_fr + delta_br) / 2;

    float delta_left_in_meters = delta_left * distance_per_tick;
    float delta_right_in_meters = delta_right * distance_per_tick;

    float distance = (delta_left_in_meters + delta_right_in_meters) / 2;
    float dTheta = (delta_right_in_meters - delta_left_in_meters) / wheelbase_m;

    x += distance * cos(theta + dTheta / 2);
    y += distance * sin(theta + dTheta / 2);
    theta += dTheta;

    prev_timestamp_ms = timestamp_ms;
    prev_fl_ticks = fl_ticks;
    prev_fr_ticks = fr_ticks;
    prev_bl_ticks = bl_ticks;
    prev_br_ticks = br_ticks;

    std::cout << timestamp_ms << ' ' << x << ' ' << y << ' ' << theta << std::endl;
  }

  if (inputFile.fail() && !inputFile.eof()) {
    std::cerr << argv[1] << " has incorrect data format." << std::endl;
    return 1;
  }

  return 0;
}
