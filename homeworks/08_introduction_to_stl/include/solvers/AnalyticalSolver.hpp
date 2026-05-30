#pragma once
#include "../Types.hpp"
#include "../interfaces/IBallisticSolver.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class AnalyticalSolver : public IBallisticSolver {
public:
  Coord solve(const Coord targetCoord, const Coord droneCoord, const float hDist) override;
  virtual ~AnalyticalSolver() = default;
};