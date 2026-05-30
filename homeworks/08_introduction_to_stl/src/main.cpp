#include <iostream>
#include <fstream>
#include <vector>
#include "third_party/json.hpp"
#include "types.hpp"
#include "Logger.hpp"
#include "MathUtils.hpp"
#include "config/ComponentFactory.hpp"
#include "MissionProcessor.hpp"

using json = nlohmann::json;

// Some rules are turned off because some thigs is not learned at this point, or requires a lot of time to refactor.
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-owning-memory)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic, readability-identifier-length)
// NOLINTBEGIN(cppcoreguidelines-special-member-functions)

void writeSimulation(const std::vector<float>& droneXHistory,
                     const std::vector<float>& droneYHistory,
                     const std::vector<float>& droneDirHistory,
                     const std::vector<DroneState>& droneStateHistory,
                     const std::vector<int>& droneSelectedTargetHistory,
                     const size_t steps)
{
  std::ofstream simulation("simulation.txt");
  simulation << steps << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneXHistory[i] << ' ' << droneYHistory[i] << ' ';
  }
  simulation << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneDirHistory[i] << ' ';
  }
  simulation << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneStateHistory[i] << ' ';
  }
  simulation << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneSelectedTargetHistory[i] << ' ';
  }
  simulation << std::endl;

  simulation.close();
}

json toJsonXY(const Coord& coord)
{
  return {{"x", coord.x}, {"y", coord.y}};
}

void writeSimulationJson(const std::vector<SimStep>& stepsLog)
{
  json out;

  out["totalSteps"] = stepsLog.size();
  out["steps"] = json::array();

  for (const SimStep& step : stepsLog) {
    json outStep;

    outStep["position"] = toJsonXY(step.pos);
    outStep["direction"] = step.direction;
    outStep["state"] = step.state;
    outStep["targetIndex"] = step.targetIdx;
    outStep["dropPoint"] = toJsonXY(step.dropPoint);
    outStep["aimPoint"] = toJsonXY(step.aimPoint);
    outStep["predictedTarget"] = toJsonXY(step.predictedTarget);

    out["steps"].push_back(outStep);
  }

  std::ofstream outJsonFile("simulation.json");
  outJsonFile << out.dump(2);
}

int main()
{
  IConfigLoader* configLoader = createLoader(LoaderType::FILE);
  const bool isConfigLoadSuccess =
    configLoader->load("homeworks/08_introduction_to_stl/config.json", "homeworks/08_introduction_to_stl/ammo.json");

  ITargetProvider* targetProvider =
    createProvider(ProviderType::JSON, "homeworks/08_introduction_to_stl/targets.json", configLoader->getConfig());
  IBallisticSolver* solver = createSolver(SolverType::ANALYTICAL);

  MissionProcessor missionProcessor{targetProvider, solver};
  const bool isInitSucces = missionProcessor.init(configLoader);
  if (!isConfigLoadSuccess || !targetProvider->isLoadSucces() || !isInitSucces) {
    return 1;
  }

  // Історія для зворотньої сумісності з .txt результатом симуляції.
  std::vector<float> droneXHistory;
  std::vector<float> droneYHistory;
  std::vector<float> droneDirHistory;
  std::vector<DroneState> droneStateHistory;
  std::vector<int> droneSelectedTargetHistory;

  while (missionProcessor.hasNext()) {
    const SimStep stepResult = missionProcessor.step();

    droneXHistory.push_back(stepResult.pos.x);
    droneYHistory.push_back(stepResult.pos.y);
    droneDirHistory.push_back(stepResult.direction);
    droneStateHistory.push_back(stepResult.state);
    droneSelectedTargetHistory.push_back(stepResult.targetIdx);
  }

  const std::vector<SimStep> stepsLog = missionProcessor.getStepsLog();

  writeSimulation(droneXHistory, droneYHistory, droneDirHistory, droneStateHistory, droneSelectedTargetHistory, stepsLog.size());

  writeSimulationJson(stepsLog);

  LOG("Simulation complete. Steps: " << stepsLog.size());

  missionProcessor.reset();

  delete configLoader;
  delete targetProvider;
  delete solver;
  configLoader = nullptr;
  targetProvider = nullptr;
  solver = nullptr;

  return 0;
}

// NOLINTEND(cppcoreguidelines-special-member-functions)
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic, readability-identifier-length)
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-owning-memory)
// NOLINTEND(bugprone-easily-swappable-parameters)