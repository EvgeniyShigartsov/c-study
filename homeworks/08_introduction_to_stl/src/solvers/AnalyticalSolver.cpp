#pragma once
#include "Types.hpp"
#include "Logger.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "MathUtils.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class AnalyticalSolver : public IBallisticSolver {
public:
  Coord solve(const Coord targetCoord, const Coord droneCoord, const float h) override
  {
    const Coord delta = targetCoord - droneCoord;
    return targetCoord - normalizeCoord(delta) * h;
  }
  virtual ~AnalyticalSolver() = default;
};