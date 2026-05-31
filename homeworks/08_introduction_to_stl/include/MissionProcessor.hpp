#pragma once
#include <vector>
#include "types.hpp"
#include "MathUtils.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class MissionProcessor {
private:
  Simulation sim;
  ITargetProvider* targetProvider;
  IBallisticSolver* ballisticSolver;

  DroneConfig dc{};
  float bombFlightTime = 0.0f;
  float h = 0.0f;
  float droneAcceleration = 0.0f;
  int targetsCount = 0;

  std::vector<SimStep> stepsLog;

public:
  MissionProcessor(ITargetProvider* provider, IBallisticSolver* solver);
  [[nodiscard]] bool init(IConfigLoader* configLoader);
  [[nodiscard]] bool hasNext() const;
  SimStep step();
  void changeSolver(IBallisticSolver* solver);
  void reset();
  std::vector<SimStep> getStepsLog();
  virtual ~MissionProcessor();
};