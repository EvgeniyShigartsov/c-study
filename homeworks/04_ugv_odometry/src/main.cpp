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

  std::ifstream file(argv[1]);

  long prev_timestamp_ms;
  long prev_fl_ticks, prev_fr_ticks;
  long prev_bl_ticks, prev_br_ticks;
  file >> prev_timestamp_ms >> prev_fl_ticks >> prev_fr_ticks >> prev_bl_ticks >> prev_br_ticks;

  long timestamp_ms;
  long fl_ticks, fr_ticks;
  long bl_ticks, br_ticks;

  bool endOfFile = false;

  while (file >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
    long delta_fl = fl_ticks - prev_fl_ticks;
    long delta_fr = fr_ticks - prev_fr_ticks;

    long delta_bl = bl_ticks - prev_bl_ticks;
    long delta_br = br_ticks - prev_br_ticks;

    long delta_left = (delta_fl + delta_bl) / 2;
    long delta_right = (delta_fr + delta_br) / 2;

    long distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;

    long delta_left_in_meters = delta_left * distance_per_tick;
    long delta_right_in_meters = delta_right * distance_per_tick;

    long distance = (delta_left_in_meters + delta_right_in_meters) / 2;
    long dTheta = (delta_right_in_meters - delta_left_in_meters) / wheelbase_m;
  }

  return 0;
}
