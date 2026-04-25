#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

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
  Coord startPos;
  float altitude;
  float initialDir;
  float v0;
  float accelerationPath;
  char ammoName[BOMB_CHAR_COUNT];
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

bool readDroneConfig (DroneConfig& out_config) {
    std::ifstream input("input.txt");

    if (!input.is_open()) {
      std::cout << "input.txt not found." << std::endl;
      return false;
  }

  input >>
    out_config.startPos.x >> out_config.startPos.y >> out_config.altitude >> out_config.initialDir >>
    out_config.v0 >> out_config.accelerationPath >> out_config.ammoName >> out_config.arrayTimeStep >>
    out_config.simTimeStep >> out_config.hitRadius >> out_config.angularSpeed >> out_config.turnThreshold;
  
   if(input.fail()){
    std::cout << "input.txt has incorrect data format." << std::endl;
    return false;
  }
  input.close();

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

float interpolateCoord (const float frac, const float currentTargetPos, const float nextTargetPos){
     return currentTargetPos + (nextTargetPos - currentTargetPos) * frac;
}

void setInterpolationIndex (
  const float t, const float arrayTimeStep,
  int& out_idx, int& out_next, float& out_frac
  ){
    out_idx = (int)(floor(t / arrayTimeStep)) % TARGET_MOVES_COUNT;
    out_next = (out_idx + 1) % TARGET_MOVES_COUNT;
    out_frac = (t - out_idx * arrayTimeStep) / arrayTimeStep;
}

float calcDistance (const float targetX, const float targetY, const float droneX, const float droneY){
  return sqrt(pow(targetX - droneX, 2) + pow(targetY - droneY, 2));
}

void setFirePoint (
  const float targetX, const float targetY, const float xd, const float yd, const float h,
  float& out_fireX, float& out_fireY
){
  float D = calcDistance(targetX, targetY, xd, yd); // Distance from drone to target
  float ratio = (D - h) / D;

  out_fireX = xd + (targetX - xd) * ratio;
  out_fireY = yd + (targetY - yd) * ratio;
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
  float& out_droneX, float& out_droneY
  ){
    out_droneX += cos(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
    out_droneY += sin(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
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
      int idx, next;
      float frac;
      setInterpolationIndex(sim.CURRENT_TIME, dc.arrayTimeStep, idx, next, frac);

      int bestTarget = 0;
      float bestTime = -1.0f;
      float bestFireX, bestFireY;
      float bestTargetX, bestTargetY;
      float actualDistX, actualDistY;

      for(int i = 0; i < TARGETS_COUNT; i++){
        const float targetCurrentX = interpolateCoord(frac, targetXInTime[i][idx], targetXInTime[i][next]);
        const float targetCurrentY = interpolateCoord(frac, targetYInTime[i][idx], targetYInTime[i][next]);

        // 1. Розрахувати орієнтовний час прильоту дрона до точки скиду (totalTime) для поточної позиції цілі
        float currentFireX, currentFireY;
        setFirePoint(targetCurrentX, targetCurrentY, sim.CURRENT_POS.x, sim.CURRENT_POS.y, h, currentFireX, currentFireY);

        const float timeToCurrentFire = calcDistance(currentFireX, currentFireY, sim.CURRENT_POS.x, sim.CURRENT_POS.y) / dc.v0 + bombFlightTime;
        
        // 2. Обчислити швидкість цілі (targetVx, targetVy) через кінцеві різниці
        int idxNext, nextNext;
        float fracNext;
        setInterpolationIndex(sim.CURRENT_TIME + dc.simTimeStep, dc.arrayTimeStep, idxNext, nextNext, fracNext);

        const float targetXNext = interpolateCoord(fracNext, targetXInTime[i][idxNext], targetXInTime[i][nextNext]);
        const float targetYNext = interpolateCoord(fracNext, targetYInTime[i][idxNext], targetYInTime[i][nextNext]);

        const float dx = targetXNext - targetCurrentX;
        const float dy = targetYNext - targetCurrentY;

        const float targetVx = dx / dc.simTimeStep;
        const float targetVy = dy / dc.simTimeStep;


        // 3. Інтерполювати прогнозовану позицію цілі на момент currentTime + totalTime
        const float targetPredictedX = targetCurrentX + targetVx * timeToCurrentFire;
        const float targetPredictedY = targetCurrentY + targetVy * timeToCurrentFire;

        // 4. Перерахувати балістику до прогнозованої позиції
        float predictedFireX, predictedFireY;
        setFirePoint(targetPredictedX, targetPredictedY, sim.CURRENT_POS.x, sim.CURRENT_POS.y, h, predictedFireX, predictedFireY);

        const float timeToPredictedFire = calcDistance(predictedFireX, predictedFireY, sim.CURRENT_POS.x, sim.CURRENT_POS.y) / dc.v0 + bombFlightTime;

        float totalTime = timeToPredictedFire;

        if(i != sim.selectedTargetIndex){
          float timeToChangeTarget = 0.0f; // STOPPED стан або deltaAngle < turnThreshold;

          const float dirToFire = atan2(predictedFireY - sim.CURRENT_POS.y, predictedFireX - sim.CURRENT_POS.x);
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
          bestFireX = predictedFireX;
          bestFireY = predictedFireY;
          bestTargetX = targetPredictedX;
          bestTargetY = targetPredictedY;
        }
      }
     
      if(!sim.needsManeuver){
        sim.selectedTargetIndex = bestTarget;
      }

      if(sim.selectedTargetIndex != sim.prevSelectedTargetIndex || sim.step == 0){
        sim.reachedManeuverPoint = false;

        float distDroneToTarget = calcDistance(sim.CURRENT_POS.x, sim.CURRENT_POS.y, bestTargetX, bestTargetY);
        float distFireToTarget = calcDistance(bestFireX, bestFireY, bestTargetX, bestTargetY);

        if(distFireToTarget > distDroneToTarget){
          // дрон між ціллю і точкою скиду - треба відлетіти далі
          const float dirAwayFromTarget = atan2(sim.CURRENT_POS.y - bestTargetY, sim.CURRENT_POS.x - bestTargetX);
          sim.needsManeuver = true;
          sim.maneuverPoint.x = sim.CURRENT_POS.x + (cos(dirAwayFromTarget) * (h + dc.accelerationPath) * 2);
          sim.maneuverPoint.y = sim.CURRENT_POS.y + (sin(dirAwayFromTarget) * (h + dc.accelerationPath) * 2);
        }
      }

      if(sim.needsManeuver && !sim.reachedManeuverPoint){
        actualDistX = sim.maneuverPoint.x;
        actualDistY = sim.maneuverPoint.y;
      } else {
        actualDistX = bestFireX;
        actualDistY = bestFireY;
      }

      if(!sim.reachedManeuverPoint && calcDistance(sim.CURRENT_POS.x, sim.CURRENT_POS.y, actualDistX, actualDistY) <= dc.hitRadius){
        sim.reachedManeuverPoint = true;
        sim.needsManeuver = false;
        actualDistX = bestFireX;
        actualDistY = bestFireY;
      }

        // Перевірено кут повороту та змінено стан відповідно вибраної цілі
      const float dirToFire = atan2(actualDistY - sim.CURRENT_POS.y, actualDistX - sim.CURRENT_POS.x);
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
        updateDroneXY(sim.CURRENT_DIR, sim.CURRENT_SPEED, dc.simTimeStep, sim.CURRENT_POS.x, sim.CURRENT_POS.y);        

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
        updateDroneXY(sim.CURRENT_DIR, sim.CURRENT_SPEED, dc.simTimeStep, sim.CURRENT_POS.x, sim.CURRENT_POS.y);

      } else if(sim.CURRENT_STATE == MOVING){
        updateDroneXY(sim.CURRENT_DIR, sim.CURRENT_SPEED, dc.simTimeStep, sim.CURRENT_POS.x, sim.CURRENT_POS.y);
      }

      if(calcDistance(sim.CURRENT_POS.x, sim.CURRENT_POS.y, bestFireX, bestFireY) <= dc.hitRadius && !sim.needsManeuver) {
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
