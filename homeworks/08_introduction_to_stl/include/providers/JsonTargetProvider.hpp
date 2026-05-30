#pragma once
#include "../Types.hpp"
#include "../interfaces/ITargetProvider.hpp"
#include <vector>

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
  JsonTargetProvider(const std::string& pathToConfig, const DroneConfig& droneConfig);

  Target getTarget(const float simCurrentTime, const int targetIndex) override;
  int getTargetCount() override { return TARGETS_COUNT; }
  bool isLoadSucces() override { return isSuccesFullyLoaded; }

  virtual ~JsonTargetProvider() = default;
};