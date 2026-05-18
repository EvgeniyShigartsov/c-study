#include "../include/ballistics.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

float get_h(float t, float d, float g, float l, float m, float v0)
{
  float l2 = pow(l, 2);
  float l2p1 = l2 + 1;

  float term1 = (pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (l2 - 1) * v0)) / (36 * pow(m, 2));

  float term2 = (pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * l2 * l2p1 * v0)) / (36 * l2p1 * pow(m, 4));

  float term3 = (pow(t, 4) * (3 * pow(d, 3) * (l2p1)*l2 * v0 + 6 * pow(d, 3) * l2p1 * pow(l, 4) * v0 -
                              6 * pow(d, 2) * g * (pow(l, 4) + l2p1) * l * m)) /
                (36 * pow(l2p1, 2) * pow(m, 3));

  float term4 = (d * pow(t, 2) * v0) / (m * 2);

  return term1 + term2 + term3 - term4 + (t * v0);
}

int calculateFirePoint(BallisticInput bi, FirePoint& out_firePoint)
{
  float m;          // ammoMass
  float d;          // coeffAero
  float l;          // liftForce
  float g = 9.81f;  // gravity

  if (strcmp(bi.ammo_name, "VOG-17") == 0) {
    m = 0.35;
    d = 0.07;
    l = 0.0;
  }
  else if (strcmp(bi.ammo_name, "M67") == 0) {
    m = 0.6;
    d = 0.10;
    l = 0.0;
  }
  else if (strcmp(bi.ammo_name, "RKG-3") == 0) {
    m = 1.2;
    d = 0.10;
    l = 0.0;
  }
  else if (strcmp(bi.ammo_name, "GLIDING-VOG") == 0) {
    m = 0.45;
    d = 0.10;
    l = 1.0;
  }
  else if (strcmp(bi.ammo_name, "GLIDING-RKG") == 0) {
    m = 1.4;
    d = 0.10;
    l = 1.0;
  }
  else {
    std::cerr << "Invalid ammo_name: " << bi.ammo_name << std::endl;
    return 1;
  }

  float a = (d * g * m) - ((pow(d, 2) * 2) * l * bi.v0);
  float b = ((-3 * g) * (pow(m, 2))) + ((d * 3) * l * m * bi.v0);
  float c = (6 * pow(m, 2)) * bi.zd;

  float p = -pow(b, 2) / (3 * pow(a, 2));
  float q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

  float angCos = 3 * q / (2 * p) * sqrt(-3 / p);

  if (angCos > 1.0f || angCos < -1.0f) {
    std::cerr << "arccos is out -1...1, value is: " << angCos << std::endl;
    return 1;
  }

  float fi = acos(angCos);

  float t = 2 * sqrt(-p / 3) * cos((fi + M_PI * 4) / 3) - b / (3 * a);
  float h = get_h(t, d, g, l, m, bi.v0);

  if (bi.xd == bi.targetX) {
    bi.xd = bi.targetX - (h + bi.accelerationPath);
  }

  float D = sqrt(pow(bi.targetX - bi.xd, 2) + pow(bi.targetY - bi.yd, 2));  // Distance from drone to target

  bool shouldMakeManeuver = h + bi.accelerationPath > D;

  float valid_xd = shouldMakeManeuver ? bi.targetX - (bi.targetX - bi.xd) * (h + bi.accelerationPath) / D : bi.xd;
  float valid_yd = shouldMakeManeuver ? bi.targetY - (bi.targetY - bi.yd) * (h + bi.accelerationPath) / D : bi.yd;
  float valid_D = shouldMakeManeuver ? sqrt(pow(bi.targetX - valid_xd, 2) + pow(bi.targetY - valid_yd, 2)) : D;

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

  return 0;
}