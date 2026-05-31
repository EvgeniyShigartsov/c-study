#include <vector>
#include "types.hpp"
#include "MathUtils.hpp"
#include "Logger.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "MissionProcessor.hpp"

const int MAX_STEPS = 10000;
const float GRAVITY = 9.81f;  // gravity

MissionProcessor::MissionProcessor(ITargetProvider* provider, IBallisticSolver* solver)
  : targetProvider(provider)
  , ballisticSolver(solver)
{
  stepsLog.reserve(MAX_STEPS);
}

[[nodiscard]] bool MissionProcessor::init(IConfigLoader* configLoader)
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

[[nodiscard]] bool MissionProcessor::hasNext() const
{
  return sim.step <= MAX_STEPS && !sim.reachedFirePoint;
}

SimStep MissionProcessor::step()
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
    dirToFire > sim.CURRENT_DIR ? sim.CURRENT_DIR += dc.angularSpeed* dc.simTimeStep : sim.CURRENT_DIR -= dc.angularSpeed * dc.simTimeStep;

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

void MissionProcessor::changeSolver(IBallisticSolver* solver)
{
  ballisticSolver = solver;
}

void MissionProcessor::reset()
{
  sim = Simulation(dc.startPos, dc.initialDir, dc.simTimeStep);
}

std::vector<SimStep> MissionProcessor::getStepsLog()
{
  return stepsLog;
};
MissionProcessor::~MissionProcessor() = default;