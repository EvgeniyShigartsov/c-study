#include <cstring>
#include <iostream>
#include <fstream>

float get_h(float t, float d, float g, float l, float m, float v0);

const int BOMBS_COUNT = 5;
const int BOMB_CHAR_COUNT = 12;
const char bombNames[BOMBS_COUNT][BOMB_CHAR_COUNT] =  {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
const float bombM[BOMBS_COUNT] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};
const float bombD[BOMBS_COUNT] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
const float bombL[BOMBS_COUNT] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

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
  input.close();

   if(input.fail()){
    std::cout << "input.txt has incorrect data format." << std::endl;
    return false;
  }

  return true;
};

int main(){

  std::ifstream targets("targets.txt");

  if (!targets.is_open()) {
    std::cout << "input.txt or targets.txt not found." << std::endl;
    return 1;
  }
  
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

  if(!readInput(
    xd, yd, zd, initialDir, v0, accelerationPath, ammo_name, arrayTimeStep, simTimeStep, hitRadius,
    angularSpeed, turnThreshold)){
      return 1;
    }
  
  float m; // ammoMass
  float d; // coeffAero
  float l; // liftForce
  const float g = 9.81f; // gravity

  float targetXInTime[TARGETS_COUNT][TARGET_MOVES_COUNT] = {};
  float targetYInTime[TARGETS_COUNT][TARGET_MOVES_COUNT] = {};


  for (int i = 0; i < BOMBS_COUNT; i++){
    if(strcmp(ammo_name, bombNames[i]) == 0){
      m = bombM[i];
      d = bombD[i];
      l = bombL[i];
      break;
    }
    if(i == BOMBS_COUNT - 1){
      std::cerr << "Invalid ammo_name: " << ammo_name << std::endl;
      return 1;
    }
  }

  for (int i = 0; i < TARGETS_COUNT; i++){
    for (int x = 0; x < TARGET_MOVES_COUNT; x++){
      targets >> targetXInTime[i][x];
    }
  }
  for(int i = 0; i < TARGETS_COUNT; i++){
    for(int y = 0; y < TARGET_MOVES_COUNT; y++){
      targets >> targetYInTime[i][y];
    }
  }

  targets.close();

  if(targets.fail()){
    std::cout << "targets.txt has incorrect data format." << std::endl;
  }

  std::ofstream simulation("simulation.txt");


 simulation << 0 << ' ' << 0 << std::endl;
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