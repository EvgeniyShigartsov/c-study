#pragma once
#include <cstring>
#include "Types.hpp"

// NOLINTBEGIN(readability-identifier-length)

bool setBombFlightTime(
  const float d, const float g, const float m, const float l, const float v0, const float zd, float& out_bombFlightTime);

float get_h(const float t, const float d, const float g, const float l, const float m, const float v0);

Coord interpolatePos(const float frac, const Coord& currentTargetPos, const Coord& nextTargetPos);
float length(const Coord& coord);

Coord normalizeCoord(const Coord& coord);

InterpolationIndex getInterpolationIndex(const float t, const float arrayTimeStep, const int targetMovesCount);

float getDirectionFromTo(const Coord& from, const Coord& to);

// NOLINTEND(readability-identifier-length)