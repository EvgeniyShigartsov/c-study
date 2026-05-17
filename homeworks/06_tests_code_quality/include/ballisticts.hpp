#pragma once

#include <string>

struct FirePoint {
  float fireX;
  float fireY;
  bool hasManeuver = false;
  float maneuverX;
  float maneuverY;
};

float get_h(float t, float d, float g, float l, float m, float v0);

int calculateFirePoint(const std::string pathToFile, FirePoint& out_firePoint);