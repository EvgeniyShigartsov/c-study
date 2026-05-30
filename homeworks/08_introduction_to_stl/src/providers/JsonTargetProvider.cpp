#include <cstring>
#include <fstream>
#include <cmath>
#include <string>
#include "third_party/json.hpp"
#include "Types.hpp"
#include "Logger.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "MathUtils.hpp"

using json = nlohmann::json;

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class JsonTargetProvider : public ITargetProvider {
private:
  bool isSuccesFullyLoaded = false;
  std::vector<std::vector<Coord>> targetsInTime;

  int TARGETS_COUNT = 0;
  int TARGET_MOVES_COUNT = 0;
  float arrayTimeStep = 0.0f;
  float simTimeStep = 0.0f;

public:
  JsonTargetProvider(const std::string& pathToConfig, const DroneConfig& droneConfig)
    : arrayTimeStep(droneConfig.arrayTimeStep)
    , simTimeStep(droneConfig.simTimeStep)
  {
    std::ifstream targetsFile(pathToConfig);

    if (!targetsFile.is_open()) {
      LOG("targets.json was not found.");
      return;
    }

    json targetsData;
    targetsFile >> targetsData;

    TARGETS_COUNT = targetsData["targetCount"];
    TARGET_MOVES_COUNT = targetsData["timeSteps"];

    targetsInTime.reserve(TARGETS_COUNT);

    try {
      for (int target = 0; target < TARGETS_COUNT; target++) {
        std::vector<Coord> targetInTime;
        targetInTime.reserve(TARGET_MOVES_COUNT);

        for (int move = 0; move < TARGET_MOVES_COUNT; move++) {
          targetInTime.push_back({
            .x = targetsData["targets"][target]["positions"][move]["x"],
            .y = targetsData["targets"][target]["positions"][move]["y"],
          });
        }

        targetsInTime.push_back(std::move(targetInTime));
      }
      isSuccesFullyLoaded = true;
    }
    catch (const json::exception& parseError) {
      LOG("targets.json parse error: " << parseError.what());
    }
  }

  Target getTarget(const float simCurrentTime, const int targetIndex) override
  {
    const InterpolationIndex currentIndex = getInterpolationIndex(simCurrentTime, arrayTimeStep, TARGET_MOVES_COUNT);

    const Coord targetCurrentXY =
      interpolatePos(currentIndex.frac, targetsInTime[targetIndex][currentIndex.idx], targetsInTime[targetIndex][currentIndex.next]);

    // 2. Обчислити швидкість цілі (targetVx, targetVy) через кінцеві різниці
    const InterpolationIndex nextIndex = getInterpolationIndex(simCurrentTime + simTimeStep, arrayTimeStep, TARGET_MOVES_COUNT);

    const Coord targetNextXY =
      interpolatePos(nextIndex.frac, targetsInTime[targetIndex][nextIndex.idx], targetsInTime[targetIndex][nextIndex.next]);
    const Coord targetVelocity = (targetNextXY - targetCurrentXY) / simTimeStep;

    return {.pos = targetCurrentXY, .velocity = targetVelocity};
  }
  int getTargetCount() override { return TARGETS_COUNT; }
  bool isLoadSucces() override { return isSuccesFullyLoaded; }

  virtual ~JsonTargetProvider() = default;
};