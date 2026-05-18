#pragma once

struct FirePoint {
  float fireX;
  float fireY;
  bool hasManeuver = false;
  float maneuverX;
  float maneuverY;
  bool success;
};

struct BallisticInput {
  float xd;
  float yd;
  float zd;
  float targetX;
  float targetY;
  float v0;
  float accelerationPath;
  char ammo_name[12];
};

float get_h(float t, float d, float g, float l, float m, float v0);

int calculateFirePoint(BallisticInput bi, FirePoint& out_firePoint);