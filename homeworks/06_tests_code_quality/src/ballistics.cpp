#include "../include/ballistics.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

// NOLINTNEXTLINE(readability-identifier-length), formula values
float get_h(float t, float d, float g, float l, float m, float v0)
{
  float l2 = powf(l, 2);
  float l2p1 = l2 + 1;

  float term1 = (powf(t, 3) * (6 * d * g * l * m - 6 * powf(d, 2) * (l2 - 1) * v0)) / (36 * powf(m, 2));

  float term2 = (powf(t, 5) * (3 * powf(d, 3) * g * powf(l, 3) * m - 3 * powf(d, 4) * l2 * l2p1 * v0)) / (36 * l2p1 * powf(m, 4));

  float term3 = (powf(t, 4) * (3 * powf(d, 3) * (l2p1)*l2 * v0 + 6 * powf(d, 3) * l2p1 * powf(l, 4) * v0 -
                               6 * powf(d, 2) * g * (powf(l, 4) + l2p1) * l * m)) /
                (36 * powf(l2p1, 2) * powf(m, 3));

  float term4 = (d * powf(t, 2) * v0) / (m * 2);

  return term1 + term2 + term3 - term4 + (t * v0);
}

int calculateFirePoint(BallisticInput bi, FirePoint& out_firePoint)
{
  // NOLINTBEGIN(readability-identifier-length) formula values
  float m = 0.0F;
  float d = 0.0F;
  float l = 0.0F;
  float g = 9.81F;
  // NOLINTEND(readability-identifier-length) formula values

  if (strcmp(static_cast<const char*>(bi.ammo_name), "VOG-17") == 0) {
    m = 0.35;
    d = 0.07;
    l = 0.0;
  }
  else if (strcmp(static_cast<const char*>(bi.ammo_name), "M67") == 0) {
    m = 0.6;
    d = 0.10;
    l = 0.0;
  }
  else if (strcmp(static_cast<const char*>(bi.ammo_name), "RKG-3") == 0) {
    m = 1.2;
    d = 0.10;
    l = 0.0;
  }
  else if (strcmp(static_cast<const char*>(bi.ammo_name), "GLIDING-VOG") == 0) {
    m = 0.45;
    d = 0.10;
    l = 1.0;
  }
  else if (strcmp(static_cast<const char*>(bi.ammo_name), "GLIDING-RKG") == 0) {
    m = 1.4;
    d = 0.10;
    l = 1.0;
  }
  else {
    std::cerr << "Invalid ammo_name: " << static_cast<const char*>(bi.ammo_name) << std::endl;
    out_firePoint.success = false;
    return 1;
  }

  // NOLINTBEGIN(readability-identifier-length) formula values
  float a = (d * g * m) - ((powf(d, 2) * 2) * l * bi.v0);
  float b = ((-3 * g) * (powf(m, 2))) + ((d * 3) * l * m * bi.v0);
  float c = (6 * powf(m, 2)) * bi.zd;

  float p = -powf(b, 2) / (3 * powf(a, 2));
  float q = (2 * powf(b, 3)) / (27 * powf(a, 3)) + c / a;
  float angCos = 3 * q / (2 * p) * sqrtf(-3 / p);
  // NOLINTEND(readability-identifier-length) formula values

  if (angCos > 1.0F || angCos < -1.0F) {
    std::cerr << "arccos is out -1...1, value is: " << angCos << std::endl;
    out_firePoint.success = false;
    return 1;
  }

  // NOLINTBEGIN(readability-identifier-length) formula values
  float fi = acosf(angCos);
  float t = 2 * sqrtf(-p / 3) * cosf((fi + static_cast<float>(M_PI) * 4) / 3) - b / (3 * a);
  float h = get_h(t, d, g, l, m, bi.v0);
  // NOLINTEND(readability-identifier-length) formula values

  if (bi.xd == bi.targetX) {
    bi.xd = bi.targetX - (h + bi.accelerationPath);
  }

  // NOLINTNEXTLINE(readability-identifier-length) formula value, distance from drone to target
  float D = sqrtf(powf(bi.targetX - bi.xd, 2) + powf(bi.targetY - bi.yd, 2));

  bool shouldMakeManeuver = h + bi.accelerationPath > D;

  float valid_xd = shouldMakeManeuver ? bi.targetX - (bi.targetX - bi.xd) * (h + bi.accelerationPath) / D : bi.xd;
  float valid_yd = shouldMakeManeuver ? bi.targetY - (bi.targetY - bi.yd) * (h + bi.accelerationPath) / D : bi.yd;
  float valid_D = shouldMakeManeuver ? sqrtf(powf(bi.targetX - valid_xd, 2) + powf(bi.targetY - valid_yd, 2)) : D;

  if (shouldMakeManeuver) {
    out_firePoint.hasManeuver = true;
    out_firePoint.maneuverX = valid_xd;
    out_firePoint.maneuverY = valid_yd;
  }

  float ratio = (valid_D - h) / valid_D;

  float fireX = valid_xd + (bi.targetX - valid_xd) * ratio;
  float fireY = valid_yd + (bi.targetY - valid_yd) * ratio;

  out_firePoint.fireX = fireX;
  out_firePoint.fireY = fireY;
  out_firePoint.success = true;

  return 0;
}