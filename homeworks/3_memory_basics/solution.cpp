#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>
#include "json.hpp"

using json = nlohmann::json;

const int BOMBS_COUNT = 5;
const int BOMB_CHAR_COUNT = 12;

const int MAX_STEPS = 10000;
const int TARGETS_COUNT = 5;
const int TARGET_MOVES_COUNT = 60;

enum DroneState { STOPPED, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Coord {
  float x;
  float y;

  Coord operator+(const Coord& other) const {
    Coord result {x + other.x, y + other.y};
    return result;
	}

  Coord operator-(const Coord& other) const {
    Coord result {x - other.x, y - other.y};
    return result;
  }
  Coord operator*(const float scalar) const {
    Coord result {x * scalar, y * scalar};
    return result;
  }
  Coord operator/(const float divider) const {
    Coord result {x / divider, y / divider};
    return result;
  }
  bool operator==(const Coord& other) const {
    return abs(x - other.x) < 1e-6f && abs(y - other.y) < 1e-6f;
  }
};

struct DroneConfig {
  char ammoName[BOMB_CHAR_COUNT];
  Coord startPos;
  float altitude;
  float initialDir;
  float v0;
  float accelerationPath;
  float arrayTimeStep;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;
};

struct BombParams {
  char name[BOMB_CHAR_COUNT];
  float mass;
  float drag;
  float lift;
};
struct SimStep {
  Coord CURRENT_POS;
  float CURRENT_TIME = 0.0f;
  float CURRENT_SPEED = 0.0f;
  float CURRENT_DIR;
  DroneState CURRENT_STATE = STOPPED;
  Coord maneuverPoint = { 0.0f, 0.0f };
  float turningTimeLeft = 0.0f;

  int selectedTargetIndex = 0;
  int prevSelectedTargetIndex = 0;
  int step = 0;
  bool reachedFirePoint = false;
  bool needsManeuver = false;
  bool reachedManeuverPoint = false;
 
  SimStep(const Coord initialPos, const float initialDir){
      CURRENT_POS = initialPos;
      CURRENT_DIR = initialDir;
  }
};

struct InterpolationIndex {
  float frac;
  int idx;
  int next;
};

bool readDroneConfig (DroneConfig& out_config) {
  std::ifstream config("config.json");

  if (!config.is_open()) {
    std::cout << "config.json not found." << std::endl;
    return false;
  }

  try {
    json data;
    config >> data;

    const char* tmp = data["ammo"].get<std::string>().c_str();
    strncpy(out_config.ammoName, tmp, BOMB_CHAR_COUNT);
    
    out_config.startPos.x = data["drone"]["position"]["x"];
    out_config.startPos.y = data["drone"]["position"]["y"];
    out_config.altitude = data["drone"]["altitude"];
    out_config.initialDir = data["drone"]["initialDirection"];
    out_config.v0 = data["drone"]["attackSpeed"];
    out_config.accelerationPath = data["drone"]["accelerationPath"];
    out_config.arrayTimeStep = data["targetArrayTimeStep"];
    out_config.simTimeStep = data["simulation"]["timeStep"];
    out_config.hitRadius = data["simulation"]["hitRadius"];
    out_config.angularSpeed = data["drone"]["angularSpeed"];
    out_config.turnThreshold = data["drone"]["turnThreshold"];
    
  } catch(const json::exception& parseError){
    std::cout << "config.json parse error: " << parseError.what() << std::endl;
    return false;
  }
  config.close();

  return true;
}

bool readTargets (
  float out_targetXInTime[TARGETS_COUNT][TARGET_MOVES_COUNT],
  float out_targetYInTime[TARGETS_COUNT][TARGET_MOVES_COUNT]
) {
  std::ifstream targets("targets.txt");

  if (!targets.is_open()) {
    std::cout << "targets.txt not found." << std::endl;
    return false;
  }

  for (int t = 0; t < TARGETS_COUNT; t++){
    for (int x = 0; x < TARGET_MOVES_COUNT; x++){
      targets >> out_targetXInTime[t][x];
    }
  }
  for(int t = 0; t < TARGETS_COUNT; t++){
    for(int y = 0; y < TARGET_MOVES_COUNT; y++){
      targets >> out_targetYInTime[t][y];
    }
  }

  if(targets.fail()){
    std::cout << "targets.txt has incorrect data format." << std::endl;
    return false;
  }

  targets.close();

  return true;
}

bool readBombParams (const char ammo_name[BOMB_CHAR_COUNT], BombParams& out_bombParams){
  const char bombNames[BOMBS_COUNT][BOMB_CHAR_COUNT] =  {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
  const float bombM[BOMBS_COUNT] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};
  const float bombD[BOMBS_COUNT] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
  const float bombL[BOMBS_COUNT] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

    for (int i = 0; i < BOMBS_COUNT; i++){
      if(strcmp(ammo_name, bombNames[i]) == 0){
        strncpy(out_bombParams.name, ammo_name, BOMB_CHAR_COUNT);
        out_bombParams.mass = bombM[i];
        out_bombParams.drag = bombD[i];
        out_bombParams.lift = bombL[i];
        break;
      }
    if(i == BOMBS_COUNT - 1){
      std::cerr << "Invalid ammo_name: " << ammo_name << std::endl;
      return false;
    }
  }

  return true;
}

float get_h(float t, float d, float g, float l, float m, float v0){
    float l2 = pow(l, 2);
    float l2p1 = l2 + 1;

    float term1 =
        (pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (l2 - 1) * v0))
        /
        (36 * pow(m, 2));

    float term2 = 
     (pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * l2 * l2p1 * v0))
     /
     (36 * l2p1 * pow(m, 4));

     float term3 = 
     (pow(t, 4) *
        (3 * pow(d, 3) * (l2p1) * l2 * v0 + 6 * pow(d, 3) * l2p1 * pow(l, 4) * v0 
        - 6 * pow(d, 2) * g * (pow(l, 4) + l2p1) * l * m)
    )
     /
     (36 * pow(l2p1, 2) * pow(m, 3));

     float term4 = (d * pow(t, 2) * v0) / (m * 2);


    return term1 + term2 + term3 - term4 + (t * v0);
}

bool setBombFlightTime (
  const float d, const float g, const float m,
  const float l, const float v0, const float zd,
  float& out_bombFlightTime
){
  float a = (d * g * m) - ((pow(d, 2) * 2) * l * v0);
  float b = ((-3 * g) * (pow(m, 2))) + ((d * 3) * l * m * v0);
  float c = (6 * pow(m, 2)) * zd;

  float p = -pow(b, 2) / (3 * pow(a, 2));
  float q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

  float angCos = 3 * q / (2 * p) * sqrt(-3 / p);

  if(angCos > 1.0f || angCos < -1.0f){
    std::cerr << "arccos is out -1...1, value is: " << angCos << std::endl;
    return false;
  }

  float fi = acos(angCos);
  
  out_bombFlightTime = 2 * sqrt(-p / 3) * cos((fi + M_PI * 4) / 3) - b / (3 * a);

  return true;
}

float interpolatePos (const float frac, const float currentTargetPos, const float nextTargetPos){
     return currentTargetPos + (nextTargetPos - currentTargetPos) * frac;
}

InterpolationIndex getInterpolationIndex (const float t, const float arrayTimeStep){
  const int idx = (int)(floor(t / arrayTimeStep)) % TARGET_MOVES_COUNT;
  const int next = (idx + 1) % TARGET_MOVES_COUNT;
  const float frac = (t - idx * arrayTimeStep) / arrayTimeStep;
  
  return {frac, idx, next};
}

float calcDistance (const Coord object, const Coord target){
  return sqrt(pow(object.x - target.x, 2) + pow(object.y - target.y, 2));
}

Coord getFirePoint (const Coord targetCoord, const Coord droneCoord, const float h){
  float D = calcDistance(targetCoord, droneCoord);
  float ratio = (D - h) / D;

  const float x = droneCoord.x + (targetCoord.x - droneCoord.x) * ratio;
  const float y = droneCoord.y + (targetCoord.y - droneCoord.y) * ratio;

  return {x, y};
}

void writeSimulation (
  const float droneXHistory[MAX_STEPS], const float droneYHistory[MAX_STEPS],
  const float droneDirHistory[MAX_STEPS], const DroneState droneStateHistory[MAX_STEPS],
  const int droneSelectedTargetHistory[MAX_STEPS], const int steps
  ){
  std::ofstream simulation("simulation.txt");
  simulation << steps << std::endl;

  for(int i = 0; i < steps; i++) simulation << droneXHistory[i] << ' ' << droneYHistory[i] << ' ';
  simulation << std::endl;

  for(int i = 0; i < steps; i++) simulation << droneDirHistory[i] << ' ';
  simulation << std::endl;

  for(int i = 0; i < steps; i++) simulation << droneStateHistory[i] << ' ';
  simulation << std::endl;

  for(int i = 0; i < steps; i++) simulation << droneSelectedTargetHistory[i] << ' ';
  simulation << std::endl;

  simulation.close();
}

void updateDroneXY (
  const float CURRENT_DIR, const float CURRENT_SPEED, const float simTimeStep,
  Coord& out_dronePosition
  ){
    out_dronePosition.x += cos(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
    out_dronePosition.y += sin(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
}

float getDirectionFromTo (const Coord& from, const Coord& to){
  return atan2(to.y - from.y, to.x - from.x);
}

int main(){
  float targetXInTime[TARGETS_COUNT][TARGET_MOVES_COUNT] = {};
  float targetYInTime[TARGETS_COUNT][TARGET_MOVES_COUNT] = {};

  const float g = 9.81f; // gravity
  float bombFlightTime;

  DroneConfig dc;
  BombParams bp;

  if(!readDroneConfig(dc)
      || !readBombParams(dc.ammoName, bp)
      || !readTargets(targetXInTime, targetYInTime)
      || !setBombFlightTime(bp.drag, g, bp.mass, bp.lift, dc.v0, dc.altitude, bombFlightTime)
    ){
    return 1;
  }

  const float h = get_h(bombFlightTime, bp.drag, g, bp.lift, bp.mass, dc.v0);
  const float droneAcceleration = pow(dc.v0, 2) / (2 * dc.accelerationPath); // (a)

  SimStep sim = SimStep(dc.startPos, dc.initialDir);

  float droneXHistory[MAX_STEPS] = {};
  float droneYHistory[MAX_STEPS] = {};
  float droneDirHistory[MAX_STEPS] = {};
  DroneState droneStateHistory[MAX_STEPS] = {};
  int droneSelectedTargetHistory[MAX_STEPS] = {};

  while (sim.step <= MAX_STEPS && !sim.reachedFirePoint){
      const InterpolationIndex currentIndex = getInterpolationIndex(sim.CURRENT_TIME, dc.arrayTimeStep);

      int bestTarget = 0;
      float bestTime = -1.0f;
      Coord bestFire;
      Coord bestTargetCoord;
      Coord actualDist;

      for(int i = 0; i < TARGETS_COUNT; i++){
        const float targetCurrentX = interpolatePos(currentIndex.frac, targetXInTime[i][currentIndex.idx], targetXInTime[i][currentIndex.next]);
        const float targetCurrentY = interpolatePos(currentIndex.frac, targetYInTime[i][currentIndex.idx], targetYInTime[i][currentIndex.next]);

        // 1. Розрахувати орієнтовний час прильоту дрона до точки скиду (totalTime) для поточної позиції цілі
        Coord currentFire = getFirePoint({ targetCurrentX, targetCurrentY }, sim.CURRENT_POS, h);
        const float timeToCurrentFire = calcDistance(currentFire, sim.CURRENT_POS) / dc.v0 + bombFlightTime;
        
        // 2. Обчислити швидкість цілі (targetVx, targetVy) через кінцеві різниці
        const InterpolationIndex nextIndex = getInterpolationIndex(sim.CURRENT_TIME + dc.simTimeStep, dc.arrayTimeStep);

        const float targetXNext = interpolatePos(nextIndex.frac, targetXInTime[i][nextIndex.idx], targetXInTime[i][nextIndex.next]);
        const float targetYNext = interpolatePos(nextIndex.frac, targetYInTime[i][nextIndex.idx], targetYInTime[i][nextIndex.next]);

        const float dx = targetXNext - targetCurrentX;
        const float dy = targetYNext - targetCurrentY;

        const float targetVx = dx / dc.simTimeStep;
        const float targetVy = dy / dc.simTimeStep;


        // 3. Інтерполювати прогнозовану позицію цілі на момент currentTime + totalTime
        const float targetPredictedX = targetCurrentX + targetVx * timeToCurrentFire;
        const float targetPredictedY = targetCurrentY + targetVy * timeToCurrentFire;

        // 4. Перерахувати балістику до прогнозованої позиції
        Coord predictedFire = getFirePoint({targetPredictedX, targetPredictedY}, sim.CURRENT_POS, h);

        const float timeToPredictedFire = calcDistance(predictedFire, sim.CURRENT_POS) / dc.v0 + bombFlightTime;

        float totalTime = timeToPredictedFire;

        if(i != sim.selectedTargetIndex){
          float timeToChangeTarget = 0.0f; // STOPPED стан або deltaAngle < turnThreshold;

          const float dirToFire = getDirectionFromTo(sim.CURRENT_POS, predictedFire);
          const float deltaAngle = abs(dirToFire - sim.CURRENT_DIR);

          if(deltaAngle > dc.turnThreshold){
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
            case STOPPED: // Немає потреби додавати час
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
        if(bestTime == -1 || totalTime < bestTime) {
          bestTime = totalTime;
          bestTarget = i;
          bestFire = predictedFire;
          bestTargetCoord = {targetPredictedX, targetPredictedY};
        }
      }
     
      if(!sim.needsManeuver){
        sim.selectedTargetIndex = bestTarget;
      }

      if(sim.selectedTargetIndex != sim.prevSelectedTargetIndex || sim.step == 0){
        sim.reachedManeuverPoint = false;

        float distDroneToTarget = calcDistance(sim.CURRENT_POS, bestTargetCoord);
        float distFireToTarget = calcDistance(bestFire, bestTargetCoord);

        if(distFireToTarget > distDroneToTarget){
          // дрон між ціллю і точкою скиду - треба відлетіти далі
          const float dirAwayFromTarget = getDirectionFromTo(bestTargetCoord, sim.CURRENT_POS);
          sim.needsManeuver = true;
          sim.maneuverPoint.x = sim.CURRENT_POS.x + (cos(dirAwayFromTarget) * (h + dc.accelerationPath) * 2);
          sim.maneuverPoint.y = sim.CURRENT_POS.y + (sin(dirAwayFromTarget) * (h + dc.accelerationPath) * 2);
        }
      }

      actualDist = sim.needsManeuver && !sim.reachedManeuverPoint ? sim.maneuverPoint : bestFire;

      if(!sim.reachedManeuverPoint && calcDistance(sim.CURRENT_POS, actualDist) <= dc.hitRadius){
        sim.reachedManeuverPoint = true;
        sim.needsManeuver = false;
        actualDist = bestFire;
      }

        // Перевірено кут повороту та змінено стан відповідно вибраної цілі
      const float dirToFire = getDirectionFromTo(sim.CURRENT_POS, actualDist);
      const float deltaAngle = abs(dirToFire - sim.CURRENT_DIR);

      if(deltaAngle > dc.turnThreshold) {
        if(sim.CURRENT_STATE == MOVING || sim.CURRENT_STATE == ACCELERATING) sim.CURRENT_STATE = DECELERATING;
        else if(sim.CURRENT_STATE == STOPPED) {
          sim.CURRENT_STATE = TURNING;
          sim.turningTimeLeft = deltaAngle / dc.angularSpeed;
        }
      } else {
        sim.CURRENT_DIR = dirToFire;
      }

      // Оновлення координати, швидкість та стан дрона відповідно до поточної фази
      if(sim.CURRENT_STATE == DECELERATING){
        sim.CURRENT_SPEED -= droneAcceleration * dc.simTimeStep;
        updateDroneXY(sim.CURRENT_DIR, sim.CURRENT_SPEED, dc.simTimeStep, sim.CURRENT_POS);        

        if(sim.CURRENT_SPEED <= 0){
          sim.CURRENT_SPEED = 0;
          sim.CURRENT_STATE = STOPPED;
          sim.turningTimeLeft = deltaAngle / dc.angularSpeed;
        }
      } else if (sim.CURRENT_STATE == STOPPED){
        if(deltaAngle > dc.turnThreshold){
          sim.CURRENT_STATE = TURNING;
        } else {
          sim.CURRENT_STATE = ACCELERATING;
        }
      } else if(sim.CURRENT_STATE == TURNING){
        dirToFire > sim.CURRENT_DIR 
        ? sim.CURRENT_DIR += dc.angularSpeed * dc.simTimeStep
        : sim.CURRENT_DIR -= dc.angularSpeed * dc.simTimeStep;

        sim.turningTimeLeft -= dc.simTimeStep;

        if(sim.turningTimeLeft <= 0){
          sim.CURRENT_DIR = dirToFire;
          sim.CURRENT_STATE = ACCELERATING;
        }
      } else if (sim.CURRENT_STATE == ACCELERATING) {
        sim.CURRENT_SPEED += droneAcceleration * dc.simTimeStep;

        if(sim.CURRENT_SPEED >= dc.v0){
          sim.CURRENT_SPEED = dc.v0;
          sim.CURRENT_STATE = MOVING;
        }
        updateDroneXY(sim.CURRENT_DIR, sim.CURRENT_SPEED, dc.simTimeStep, sim.CURRENT_POS);

      } else if(sim.CURRENT_STATE == MOVING){
        updateDroneXY(sim.CURRENT_DIR, sim.CURRENT_SPEED, dc.simTimeStep, sim.CURRENT_POS);
      }

      if(calcDistance(sim.CURRENT_POS, bestFire) <= dc.hitRadius && !sim.needsManeuver) {
        sim.reachedFirePoint = true;
      }

      droneXHistory[sim.step] = sim.CURRENT_POS.x;
      droneYHistory[sim.step] = sim.CURRENT_POS.y;
      droneDirHistory[sim.step] = sim.CURRENT_DIR;
      droneStateHistory[sim.step] = sim.CURRENT_STATE;
      droneSelectedTargetHistory[sim.step] = sim.selectedTargetIndex;

      sim.prevSelectedTargetIndex = sim.selectedTargetIndex;
      sim.CURRENT_TIME += dc.simTimeStep;
      sim.step++;
  }

  writeSimulation(droneXHistory, droneYHistory, droneDirHistory, droneStateHistory, droneSelectedTargetHistory, sim.step);

  return 0;
}
