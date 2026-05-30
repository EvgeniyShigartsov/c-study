#pragma once
#include "../Types.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class IBallisticSolver {
public:
  virtual Coord solve(const Coord targetCoord, const Coord droneCoord, const float hDist) = 0;
  virtual ~IBallisticSolver() = default;
};