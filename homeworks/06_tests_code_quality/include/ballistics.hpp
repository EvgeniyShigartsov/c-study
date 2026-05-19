#pragma once

struct FirePoint {
  float fireX = 0.0F;
  float fireY = 0.0F;
  bool hasManeuver = false;
  float maneuverX = 0.0F;
  float maneuverY = 0.0F;
  bool success = false;
};

struct BallisticInput {
  float xd = 0.0F;
  float yd = 0.0F;
  float zd = 0.0F;
  float targetX = 0.0F;
  float targetY = 0.0F;
  float v0 = 0.0F;
  float accelerationPath = 0.0F;
  char ammo_name[12] = "";  // NOLINT(cppcoreguidelines-avoid-c-arrays, modernize-avoid-c-arrays), STL is not learned at this point.
};

// NOLINTNEXTLINE(readability-identifier-length), formula values.
float get_h(float t, float d, float g, float l, float m, float v0);

int calculateFirePoint(BallisticInput bi, FirePoint& out_firePoint);