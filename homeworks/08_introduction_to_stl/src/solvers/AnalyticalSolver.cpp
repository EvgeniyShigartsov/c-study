#include "Types.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "MathUtils.hpp"

Coord AnalyticalSolver::solve(const Coord targetCoord, const Coord droneCoord, const float hDist)
{
  const Coord delta = targetCoord - droneCoord;
  return targetCoord - normalizeCoord(delta) * hDist;
}

AnalyticalSolver::~AnalyticalSolver() = default;