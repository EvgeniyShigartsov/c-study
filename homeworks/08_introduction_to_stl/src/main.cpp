#include <cstddef>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <map>
#include "../third_party/json.hpp"
#include "../include/types.hpp"
#include "../include/Logger.hpp"
#include "../include/MathUtils.hpp"

using json = nlohmann::json;

// Some rules are turned off because some thigs is not learned at this point, or requires a lot of time to refactor.
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-owning-memory)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic, readability-identifier-length)
// NOLINTBEGIN(cppcoreguidelines-special-member-functions)

const int MAX_STEPS = 10000;
const float GRAVITY = 9.81f;  // gravity

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Target getTarget(const float simCurrentTime, const int targetIndex) = 0;
  virtual bool isLoadSucces() = 0;
  virtual ~ITargetProvider() = default;
};

class IBallisticSolver {
public:
  virtual Coord solve(const Coord targetCoord, const Coord droneCoord, const float h) = 0;
  virtual ~IBallisticSolver() = default;
};

class IConfigLoader {
public:
  virtual bool load(const std::string& droneConfigPath, const std::string& bombParamsPath) = 0;
  virtual DroneConfig getConfig() = 0;
  virtual BombParams getAmmoParams() = 0;
  virtual ~IConfigLoader() = default;
};

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

class AnalyticalSolver : public IBallisticSolver {
public:
  Coord solve(const Coord targetCoord, const Coord droneCoord, const float h) override
  {
    const Coord delta = targetCoord - droneCoord;
    return targetCoord - normalizeCoord(delta) * h;
  }
  virtual ~AnalyticalSolver() = default;
};

class FileConfigLoader : public IConfigLoader {
private:
  DroneConfig droneConfig{};
  BombParams bombParams{};

  bool readDroneConfig(const std::string& pathToConfig)
  {
    std::ifstream config(pathToConfig);

    if (!config.is_open()) {
      LOG("drone config file not found. Given path: " << pathToConfig);
      return false;
    }

    try {
      json data;
      config >> data;

      droneConfig.ammoName = data["ammo"];
      droneConfig.startPos.x = data["drone"]["position"]["x"];
      droneConfig.startPos.y = data["drone"]["position"]["y"];
      droneConfig.altitude = data["drone"]["altitude"];
      droneConfig.initialDir = data["drone"]["initialDirection"];
      droneConfig.v0 = data["drone"]["attackSpeed"];
      droneConfig.accelerationPath = data["drone"]["accelerationPath"];
      droneConfig.arrayTimeStep = data["targetArrayTimeStep"];
      droneConfig.simTimeStep = data["simulation"]["timeStep"];
      droneConfig.hitRadius = data["simulation"]["hitRadius"];
      droneConfig.angularSpeed = data["drone"]["angularSpeed"];
      droneConfig.turnThreshold = data["drone"]["turnThreshold"];
    }
    catch (const json::exception& parseError) {
      LOG("config.json parse error: " << parseError.what());
      return false;
    }
    config.close();

    return true;
  }

  bool readBombParams(const std::string& bombParamsPath)
  {
    std::ifstream ammoFile(bombParamsPath);

    if (!ammoFile.is_open()) {
      LOG("bomb params file not found. Given path: " << bombParamsPath);
      return false;
    }

    json ammoData;
    ammoFile >> ammoData;

    std::map<std::string, BombParams> ammoMap;

    try {
      for (const auto& item : ammoData) {
        const std::string ammoName = item["name"];
        ammoMap[ammoName] = {
          .name = ammoName,
          .mass = item["mass"],
          .drag = item["drag"],
          .lift = item["lift"],
        };
      }
    }
    catch (const json::exception& parseError) {
      LOG(bombParamsPath << " parse error: " << parseError.what());
    }

    auto it = ammoMap.find(droneConfig.ammoName);

    if (it == ammoMap.end()) {
      LOG("Invalid ammo_name: " << droneConfig.ammoName);
      return false;
    }

    bombParams = it->second;

    return true;
  }

public:
  bool load(const std::string& pathToConfig, const std::string& bombParamsPath) override
  {
    const bool isConfigLoadOk = readDroneConfig(pathToConfig);
    const bool isBombParamsLoadOk = readBombParams(bombParamsPath);

    if (isConfigLoadOk && isBombParamsLoadOk) {
      LOG("Config loaded: speed=" << droneConfig.v0);
      LOG("Ammo found: " << bombParams.name);
    }

    return isConfigLoadOk && isBombParamsLoadOk;
  }

  DroneConfig getConfig() override { return droneConfig; }
  BombParams getAmmoParams() override { return bombParams; }
  virtual ~FileConfigLoader() = default;
};

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
  MissionProcessor(ITargetProvider* provider, IBallisticSolver* solver)
    : targetProvider(provider)
    , ballisticSolver(solver)
  {
    stepsLog.reserve(MAX_STEPS);
  }
  [[nodiscard]] bool init(IConfigLoader* configLoader)
  {
    dc = configLoader->getConfig();
    const BombParams bp = configLoader->getAmmoParams();

    const bool IS_BOMB_OK = setBombFlightTime(bp.drag, GRAVITY, bp.mass, bp.lift, dc.v0, dc.altitude, bombFlightTime);

    sim = Simulation(dc.startPos, dc.initialDir, dc.simTimeStep);
    h = get_h(bombFlightTime, bp.drag, GRAVITY, bp.lift, bp.mass, dc.v0);
    droneAcceleration = powf(dc.v0, 2) / (2 * dc.accelerationPath);  // (a)
    targetsCount = targetProvider->getTargetCount();

    return IS_BOMB_OK;
  }
  [[nodiscard]] bool hasNext() const { return sim.step <= MAX_STEPS && !sim.reachedFirePoint; }
  SimStep step()
  {
    int bestTarget = 0;
    float bestTime = -1.0f;
    Coord bestFire{};
    Coord bestTargetPredictedXY{};
    Coord actualDist{};

    for (int i = 0; i < targetsCount; i++) {
      const Target target = targetProvider->getTarget(sim.CURRENT_TIME, i);

      // 1. Розрахувати орієнтовний час прильоту дрона до точки скиду (totalTime) для поточної позиції цілі
      Coord currentFire = ballisticSolver->solve(target.pos, sim.CURRENT_POS, h);
      const float timeToCurrentFire = length(currentFire - sim.CURRENT_POS) / dc.v0 + bombFlightTime;

      // 2. Обчислити швидкість цілі (targetVx, targetVy) через кінцеві різниці
      // Дані вже у target.velocity

      // 3. Інтерполювати прогнозовану позицію цілі на момент currentTime + totalTime
      const Coord targetPredictedXY = target.pos + target.velocity * timeToCurrentFire;

      // 4. Перерахувати балістику до прогнозованої позиції
      Coord predictedFire = ballisticSolver->solve(targetPredictedXY, sim.CURRENT_POS, h);
      const float timeToPredictedFire = length(predictedFire - sim.CURRENT_POS) / dc.v0 + bombFlightTime;

      float totalTime = timeToPredictedFire;

      if (i != sim.selectedTargetIndex) {
        float timeToChangeTarget = 0.0f;  // STOPPED стан або deltaAngle < turnThreshold;

        const float dirToFire = getDirectionFromTo(sim.CURRENT_POS, predictedFire);
        const float deltaAngle = fabsf(dirToFire - sim.CURRENT_DIR);

        if (deltaAngle > dc.turnThreshold) {
          const float turningTime = deltaAngle / dc.angularSpeed;

          // Додавання часу залежно від поточної дії дрону
          switch (sim.CURRENT_STATE) {
            case ACCELERATING:
            case DECELERATING:
              timeToChangeTarget += sim.CURRENT_SPEED / droneAcceleration;
              break;
            case MOVING:
              timeToChangeTarget += dc.v0 / droneAcceleration;
              break;
            case TURNING:
              timeToChangeTarget += sim.turningTimeLeft;
              break;
            case STOPPED:  // Немає потреби додавати час
              break;
          }
          // Додавання часу повороту
          timeToChangeTarget += turningTime;
          // Додавання часу на розгін
          timeToChangeTarget += dc.v0 / droneAcceleration;
        }
        totalTime += timeToChangeTarget;
      }

      // Обрано ціль з мінімальним загальним часом
      if (bestTime == -1 || totalTime < bestTime) {
        bestTime = totalTime;
        bestTarget = i;
        bestFire = predictedFire;
        bestTargetPredictedXY = targetPredictedXY;
      }
    }

    if (!sim.needsManeuver) {
      sim.selectedTargetIndex = bestTarget;
    }

    if (sim.selectedTargetIndex != sim.prevSelectedTargetIndex || sim.step == 0) {
      sim.reachedManeuverPoint = false;

      float distDroneToTarget = length(sim.CURRENT_POS - bestTargetPredictedXY);
      float distFireToTarget = length(bestFire - bestTargetPredictedXY);

      if (distFireToTarget > distDroneToTarget) {
        // дрон між ціллю і точкою скиду - треба відлетіти далі
        const float dirAwayFromTarget = getDirectionFromTo(bestTargetPredictedXY, sim.CURRENT_POS);
        sim.needsManeuver = true;
        sim.maneuverPoint.x = sim.CURRENT_POS.x + (cosf(dirAwayFromTarget) * (h + dc.accelerationPath) * 2);
        sim.maneuverPoint.y = sim.CURRENT_POS.y + (sinf(dirAwayFromTarget) * (h + dc.accelerationPath) * 2);
      }
    }

    actualDist = sim.needsManeuver && !sim.reachedManeuverPoint ? sim.maneuverPoint : bestFire;

    if (!sim.reachedManeuverPoint && length(sim.CURRENT_POS - actualDist) <= dc.hitRadius) {
      sim.reachedManeuverPoint = true;
      sim.needsManeuver = false;
      actualDist = bestFire;
    }

    // Перевірено кут повороту та змінено стан відповідно вибраної цілі
    const float dirToFire = getDirectionFromTo(sim.CURRENT_POS, actualDist);
    const float deltaAngle = fabsf(dirToFire - sim.CURRENT_DIR);

    if (deltaAngle > dc.turnThreshold) {
      if (sim.CURRENT_STATE == MOVING || sim.CURRENT_STATE == ACCELERATING) {
        sim.CURRENT_STATE = DECELERATING;
      }

      else if (sim.CURRENT_STATE == STOPPED) {
        sim.CURRENT_STATE = TURNING;
        sim.turningTimeLeft = deltaAngle / dc.angularSpeed;
      }
    }
    else {
      sim.CURRENT_DIR = dirToFire;
    }

    // Оновлення координати, швидкість та стан дрона відповідно до поточної фази
    if (sim.CURRENT_STATE == DECELERATING) {
      sim.CURRENT_SPEED -= droneAcceleration * dc.simTimeStep;
      sim.updateDroneXY();

      if (sim.CURRENT_SPEED <= 0) {
        sim.CURRENT_SPEED = 0;
        sim.CURRENT_STATE = STOPPED;
        sim.turningTimeLeft = deltaAngle / dc.angularSpeed;
      }
    }
    else if (sim.CURRENT_STATE == STOPPED) {
      if (deltaAngle > dc.turnThreshold) {
        sim.CURRENT_STATE = TURNING;
      }
      else {
        sim.CURRENT_STATE = ACCELERATING;
      }
    }
    else if (sim.CURRENT_STATE == TURNING) {
      dirToFire > sim.CURRENT_DIR ? sim.CURRENT_DIR += dc.angularSpeed* dc.simTimeStep
                                  : sim.CURRENT_DIR -= dc.angularSpeed * dc.simTimeStep;

      sim.turningTimeLeft -= dc.simTimeStep;

      if (sim.turningTimeLeft <= 0) {
        sim.CURRENT_DIR = dirToFire;
        sim.CURRENT_STATE = ACCELERATING;
      }
    }
    else if (sim.CURRENT_STATE == ACCELERATING) {
      sim.CURRENT_SPEED += droneAcceleration * dc.simTimeStep;

      if (sim.CURRENT_SPEED >= dc.v0) {
        sim.CURRENT_SPEED = dc.v0;
        sim.CURRENT_STATE = MOVING;
      }
      sim.updateDroneXY();
    }
    else if (sim.CURRENT_STATE == MOVING) {
      sim.updateDroneXY();
    }

    if (length(sim.CURRENT_POS - bestFire) <= dc.hitRadius && !sim.needsManeuver) {
      sim.reachedFirePoint = true;
    }

    DEBUG("Step " << sim.step << " pos=(" << sim.CURRENT_POS.x << "," << sim.CURRENT_POS.y << ")");
    DEBUG("  target=" << sim.selectedTargetIndex << " state=" << sim.CURRENT_STATE);

    const Coord dir = {cosf(sim.CURRENT_DIR), sinf(sim.CURRENT_DIR)};
    const Target predictedTarget = targetProvider->getTarget(sim.CURRENT_TIME + bombFlightTime, sim.selectedTargetIndex);

    const SimStep stepResult = {
      .pos = sim.CURRENT_POS,
      .dropPoint = bestFire,
      .aimPoint = sim.CURRENT_POS + dir * h,
      .predictedTarget = predictedTarget.pos,
      .direction = sim.CURRENT_DIR,
      .state = sim.CURRENT_STATE,
      .targetIdx = sim.selectedTargetIndex,
      .step = sim.step,
    };

    stepsLog.push_back(stepResult);

    sim.prevSelectedTargetIndex = sim.selectedTargetIndex;
    sim.CURRENT_TIME += dc.simTimeStep;
    sim.step++;

    return stepResult;
  }
  void changeSolver(IBallisticSolver* solver) { ballisticSolver = solver; }
  void reset() { sim = Simulation(dc.startPos, dc.initialDir, dc.simTimeStep); }
  std::vector<SimStep> getStepsLog() { return stepsLog; }
  virtual ~MissionProcessor() = default;
};

IConfigLoader* createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return new FileConfigLoader;
    default:
      return nullptr;
  }
}

IBallisticSolver* createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return new AnalyticalSolver;
    default:
      return nullptr;
  }
}

ITargetProvider* createProvider(ProviderType type, const char* param, const DroneConfig& droneConfig)
{
  switch (type) {
    case ProviderType::JSON:
      return new JsonTargetProvider{param, droneConfig};
    default:
      return nullptr;
  }
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