#pragma once
#include "../../include/Types.hpp"
#include "../../include/Logger.hpp"
#include "../../include/interfaces/IBallisticSolver.hpp"

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