#pragma once
#include <cmath>

struct Coord {
  float x;
  float y;

  Coord operator+(const Coord& other) const
  {
    Coord result{x + other.x, y + other.y};
    return result;
  }

  Coord operator-(const Coord& other) const
  {
    Coord result{x - other.x, y - other.y};
    return result;
  }
  Coord operator*(const float scalar) const
  {
    Coord result{x * scalar, y * scalar};
    return result;
  }
  Coord operator/(const float divider) const
  {
    Coord result{x / divider, y / divider};
    return result;
  }
  bool operator==(const Coord& other) const { return fabsf(x - other.x) < 1e-6f && fabsf(y - other.y) < 1e-6f; }
};