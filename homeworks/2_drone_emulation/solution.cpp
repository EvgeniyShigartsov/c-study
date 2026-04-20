#include <cstring>
#include <iostream>
#include <fstream>

float get_h(float t, float d, float g, float l, float m, float v0);

const int BOMBS_COUNT = 5;
const int BOMB_CHAR_COUNT = 12;

const int MAX_STEPS = 10000;
const int TARGETS_COUNT = 5;
const int TARGET_MOVES_COUNT = 60;

enum DroneState { STOPPED, ACCELERATING, DECELERATING, TURNING, MOVING };

bool readInput (
  float& out_xd,
  float& out_yd,
  float& out_zd,
  float& out_initialDir,
  float& out_v0,
  float& out_accelerationPath,
  char out_ammo_name[BOMB_CHAR_COUNT],
  float& out_arrayTimeStep,
  float& out_simTimeStep,
  float& out_hitRadius,
  float& out_angularSpeed,
  float& out_turnThreshold
) {
   std::ifstream input("input.txt");

    if (!input.is_open()) {
    std::cout << "input.txt not found." << std::endl;
    return false;
  }

  input >> out_xd >> out_yd >> out_zd >> out_initialDir >> out_v0 >> out_accelerationPath >> out_ammo_name
  >> out_arrayTimeStep >> out_simTimeStep >> out_hitRadius >> out_angularSpeed >> out_turnThreshold;
  
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

bool setBombParams (const char ammo_name[BOMB_CHAR_COUNT], float& out_m, float& out_d, float& out_l){

  const char bombNames[BOMBS_COUNT][BOMB_CHAR_COUNT] =  {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
  const float bombM[BOMBS_COUNT] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};
  const float bombD[BOMBS_COUNT] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
  const float bombL[BOMBS_COUNT] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

    for (int i = 0; i < BOMBS_COUNT; i++){
    if(strcmp(ammo_name, bombNames[i]) == 0){
      out_m = bombM[i];
      out_d = bombD[i];
      out_l = bombL[i];
      break;
    }
    if(i == BOMBS_COUNT - 1){
      std::cerr << "Invalid ammo_name: " << ammo_name << std::endl;
      return false;
    }
  }

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
  const float targetX, const float targetY, float xd, const float yd,
  const float h, const float accelerationPath,
  float& out_fireX, float& out_fireY
){
  
  float D = calcDistance(targetX, targetY, xd, yd); // Distance from drone to target

  if(D == 0){
    xd = targetX - (h + accelerationPath);
    D = calcDistance(targetX, targetY, xd, yd);
  }

  bool shouldMakeManeuver = h + accelerationPath > D;

  float valid_xd = shouldMakeManeuver ? targetX - (targetX - xd) * (h + accelerationPath) / D : xd;
  float valid_yd = shouldMakeManeuver ? targetY - (targetY - yd) * (h + accelerationPath) / D : yd;
  float valid_D = shouldMakeManeuver ? calcDistance(targetX, targetY, valid_xd, valid_yd) : D;

  float ratio = (valid_D - h) / valid_D;

  out_fireX = valid_xd + (targetX - valid_xd) * ratio;
  out_fireY = valid_yd + (targetY - valid_yd) * ratio;
}

int main(){

  float xd;
  float yd;
  float zd;
  float initialDir;
  float v0;
  float accelerationPath;
  char ammo_name[BOMB_CHAR_COUNT];
  float arrayTimeStep;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;

  float targetXInTime[TARGETS_COUNT][TARGET_MOVES_COUNT] = {};
  float targetYInTime[TARGETS_COUNT][TARGET_MOVES_COUNT] = {};

  float m; // ammoMass
  float d; // coeffAero
  float l; // liftForce
  const float g = 9.81f; // gravity

  if(!readInput(
    xd, yd, zd, initialDir, v0, accelerationPath, ammo_name, arrayTimeStep, simTimeStep, hitRadius,
    angularSpeed, turnThreshold)){
    return 1;
  }

  if(!readTargets(targetXInTime, targetYInTime)){
    return 1;
  }

  if(!setBombParams(ammo_name, m, d, l)){
    return 1;
  }

  // ==== from HM-1 start

  float bombA = (d * g * m) - ((pow(d, 2) * 2) * l * v0);
  float b = ((-3 * g) * (pow(m, 2))) + ((d * 3) * l * m * v0);
  float c = (6 * pow(m, 2)) * zd;

  float p = -pow(b, 2) / (3 * pow(bombA, 2));
  float q = (2 * pow(b, 3)) / (27 * pow(bombA, 3)) + c / bombA;

  float angCos = 3 * q / (2 * p) * sqrt(-3 / p);

  if(angCos > 1.0f || angCos < -1.0f){
    std::cerr << "arccos is out -1...1, value is: " << angCos << std::endl;
    return 1;
  }

  float fi = acos(angCos);
  
  float bombFlightTime = 2 * sqrt(-p / 3) * cos((fi + M_PI * 4) / 3) - b / (3 * bombA);
  float h = get_h(bombFlightTime, d, g, l, m, v0);

  // ==== from HM-1 end

  int step = 0;
  bool reachedFirePoint = false;
  float t = 0.0f;
  float droneX = xd;
  float droneY = yd;
  float CURRENT_DIR = initialDir;
  DroneState CURRENT_STATE = STOPPED;
  float CURRENT_SPEED = 0;
  float turningTimeLeft = 0.0f;

  float droneAcceleration = pow(v0, 2) / (2 * accelerationPath); // (a)
  int selectedTargetIndex = 0;

  float droneXHistory[MAX_STEPS] = {};
  float droneYHistory[MAX_STEPS] = {};
  float droneDirHistory[MAX_STEPS] = {};
  DroneState droneStateHistory[MAX_STEPS] = {};
  int droneSelectedTargetHistory[MAX_STEPS] = {};

  while (step <= MAX_STEPS && !reachedFirePoint){
      int idx, next;
      float frac;
      setInterpolationIndex(t, arrayTimeStep, idx, next, frac);

      float bestTime = -1;
      int bestTarget = 0;
      float bestFireX, bestFireY;


      for(int i = 0; i < TARGETS_COUNT; i++){
        float targetCurrentX = interpolateCoord(frac, targetXInTime[i][idx], targetXInTime[i][next]);
        float targetCurrentY = interpolateCoord(frac, targetYInTime[i][idx], targetYInTime[i][next]);

        // 1. Розрахувати орієнтовний час прильоту дрона до точки скиду (totalTime) для поточної позиції цілі
        float currentFireX, currentFireY;
        setFirePoint(targetCurrentX, targetCurrentY, droneX, droneY, h, accelerationPath, currentFireX, currentFireY);

        float timeToCurrentFire = calcDistance(currentFireX, currentFireY, droneX, droneY) / v0 + bombFlightTime;
        
        // 2. Обчислити швидкість цілі (targetVx, targetVy) через кінцеві різниці
        int idxNext, nextNext;
        float fracNext;
        setInterpolationIndex(t + simTimeStep, arrayTimeStep, idxNext, nextNext, fracNext);

        float targetXNext = interpolateCoord(fracNext, targetXInTime[i][idxNext], targetXInTime[i][nextNext]);
        float targetYNext = interpolateCoord(fracNext, targetYInTime[i][idxNext], targetYInTime[i][nextNext]);

        float dx = targetXNext - targetCurrentX;
        float dy = targetYNext - targetCurrentY;

        float targetVx = dx / simTimeStep;
        float targetVy = dy / simTimeStep;


        // 3. Інтерполювати прогнозовану позицію цілі на момент currentTime + totalTime
        float targetPredictedX = targetCurrentX + targetVx * timeToCurrentFire;
        float targetPredictedY = targetCurrentY + targetVy * timeToCurrentFire;

        // 4. Перерахувати балістику до прогнозованої позиції
        float predictedFireX, predictedFireY;
        setFirePoint(targetPredictedX, targetPredictedY, droneX, droneY, h, accelerationPath, predictedFireX, predictedFireY);

        float timeToPredictedFire = calcDistance(predictedFireX, predictedFireY, droneX, droneY) / v0 + bombFlightTime;

        float totalTime = timeToPredictedFire;

        if(i != selectedTargetIndex){
          float timeToChangeTarget = 0.0f; // STOPPED drone case or deltaAngle < turnThreshold;

          float dirToFire = atan2(predictedFireY - droneY, predictedFireX - droneX);
          float deltaAngle = abs(dirToFire - CURRENT_DIR);

          if(deltaAngle > turnThreshold){
            float turningTime = deltaAngle / angularSpeed;

            // Додавання часу залежно від поточної дії дрону
            switch (CURRENT_STATE) {
            case ACCELERATING:
            case DECELERATING:
              timeToChangeTarget += CURRENT_SPEED / droneAcceleration;
              break;
            case MOVING:
              timeToChangeTarget += v0 / droneAcceleration;
              break;
            case TURNING:
              timeToChangeTarget += turningTimeLeft;
              break;
            case STOPPED: // Немає потреби додавати час
              break;
          }
            // Додавання часу повороту
            timeToChangeTarget += turningTime;
            // Додавання часу на розгін
            timeToChangeTarget += v0 / droneAcceleration;
          }
          totalTime += timeToChangeTarget;
        }

        // Обрано ціль з мінімальним загальним часом
        if(bestTime == -1 || totalTime < bestTime) {
          bestTime = totalTime;
          bestTarget = i;
          bestFireX = predictedFireX;
          bestFireY = predictedFireY;
        }
      }
      selectedTargetIndex = bestTarget;

      // Перевірено кут повороту та змінено стан відповідно вибраної цілі
      float dirToFire = atan2(bestFireY - droneY, bestFireX - droneX);
      float deltaAngle = abs(dirToFire - CURRENT_DIR);

      if(deltaAngle > turnThreshold) {
        if(CURRENT_STATE == MOVING || CURRENT_STATE == ACCELERATING) CURRENT_STATE = DECELERATING;
        else if(CURRENT_STATE == STOPPED) {
          CURRENT_STATE = TURNING;
          turningTimeLeft = deltaAngle / angularSpeed;
        }
      } else {
        CURRENT_DIR = dirToFire;
      }

      // Оновлення координати, швидкість та стан дрона відповідно до поточної фази
      if(CURRENT_STATE == DECELERATING){
        CURRENT_SPEED -= droneAcceleration * simTimeStep;
        
        droneX += cos(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
        droneY += sin(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;

        if(CURRENT_SPEED <= 0){
          CURRENT_SPEED = 0;
          CURRENT_STATE = STOPPED;
          turningTimeLeft = deltaAngle / angularSpeed;
        }
      } else if (CURRENT_STATE == STOPPED){
        if(deltaAngle > turnThreshold){
          CURRENT_STATE = TURNING;
        } else {
          CURRENT_STATE = ACCELERATING;
        }
      } else if(CURRENT_STATE == TURNING){
        dirToFire > CURRENT_DIR 
        ? CURRENT_DIR += angularSpeed * simTimeStep
        : CURRENT_DIR -= angularSpeed * simTimeStep;

        turningTimeLeft -= simTimeStep;

        if(turningTimeLeft <= 0){
          CURRENT_DIR = dirToFire;
          CURRENT_STATE = ACCELERATING;
        }
      } else if (CURRENT_STATE == ACCELERATING) {
        CURRENT_SPEED += droneAcceleration * simTimeStep;

        if(CURRENT_SPEED >= v0){
          CURRENT_SPEED = v0;
          CURRENT_STATE = MOVING;
        }
        droneX += cos(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
        droneY += sin(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
      } else if(CURRENT_STATE == MOVING){
        droneX += cos(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
        droneY += sin(CURRENT_DIR) * CURRENT_SPEED * simTimeStep;
      }

      if(calcDistance(droneX, droneY, bestFireX, bestFireY) <= hitRadius){
        reachedFirePoint = true;
      }

      droneXHistory[step] = droneX;
      droneYHistory[step] = droneY;
      droneDirHistory[step] = CURRENT_DIR;
      droneStateHistory[step] = CURRENT_STATE;
      droneSelectedTargetHistory[step] = selectedTargetIndex;

      t+= simTimeStep;
      step++;
  }

  std::ofstream simulation("simulation.txt");

  simulation << step << std::endl;

  for(int i = 0; i < step; i++){
    simulation << droneXHistory[i] << ' ' << droneYHistory[i] << ' ';
  }
  simulation << std::endl;

  for(int i = 0; i < step; i++){
    simulation << droneDirHistory[i] << ' ';
  }
  simulation << std::endl;

  for(int i = 0; i < step; i++){
    simulation << droneStateHistory[i] << ' ';
  }
  simulation << std::endl;

  for(int i = 0; i < step; i++){
    simulation << droneSelectedTargetHistory[i] << ' ';
  }
  simulation << std::endl;

  simulation.close();

  return 0;
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