#include <cstring>
#include <iostream>
#include <fstream>

float get_h(float t, float d, float g, float l, float m, float v0);

const int MAX_STEPS = 10000;
const int TARGETS_COUNT = 5;
const int TARGET_MOVES_COUNT = 60;

enum DroneState { STOPPED, ACCELERATING, DECELERATING, TURNING, MOVING };

int main(){

  std::ifstream input("input.txt");

  float xd;
  float yd;
  float zd;
  float targetX;
  float targetY;
  float v0;
  float accelerationPath;
  char ammo_name[12];
  
  input >> xd >> yd >> zd >> targetX >> targetY >> v0 >> accelerationPath >> ammo_name;
  input.close();

  float m; // ammoMass
  float d; // coeffAero
  float l; // liftForce
  float g = 9.81f; // gravity

if (strcmp(ammo_name, "VOG-17") == 0) {
    m = 0.35;
    d = 0.07;
    l = 0.0;
} else if (strcmp(ammo_name, "M67") == 0) {
    m = 0.6;
    d = 0.10;
    l = 0.0;
} else if (strcmp(ammo_name, "RKG-3") == 0) {
    m = 1.2;
    d = 0.10;
    l = 0.0;
} else if (strcmp(ammo_name, "GLIDING-VOG") == 0) {
    m = 0.45;
    d = 0.10;
    l = 1.0;
} else if (strcmp(ammo_name, "GLIDING-RKG") == 0) {
    m = 1.4;
    d = 0.10;
    l = 1.0;
} else {
    std::cerr << "Invalid ammo_name: " << ammo_name << std::endl;
    return 1;
}

  float a = (d * g * m) - ((pow(d, 2) * 2) * l * v0);
  float b = ((-3 * g) * (pow(m, 2))) + ((d * 3) * l * m * v0);
  float c = (6 * pow(m, 2)) * zd;

  float p = -pow(b, 2) / (3 * pow(a, 2));
  float q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

  float angCos = 3 * q / (2 * p) * sqrt(-3 / p);

  if(angCos > 1.0f || angCos < -1.0f){
    std::cerr << "arccos is out -1...1, value is: " << angCos << std::endl;
    return 1;
  }

  float fi = acos(angCos);
  
  float t = 2 * sqrt(-p / 3) * cos((fi + M_PI * 4) / 3) - b / (3 * a);
  float h = get_h(t, d, g, l, m, v0);

  if(xd == targetX){
    xd = targetX - (h + accelerationPath);
  }

  float D = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2)); // Distance from drone to target

  bool shouldMakeManeuver = h + accelerationPath > D;

  float valid_xd = shouldMakeManeuver ? targetX - (targetX - xd) * (h + accelerationPath) / D : xd;
  float valid_yd = shouldMakeManeuver ? targetY - (targetY - yd) * (h + accelerationPath) / D : yd;
  float valid_D = shouldMakeManeuver ? sqrt(pow(targetX - valid_xd, 2) + pow(targetY - valid_yd, 2)) : D;

  std::ofstream output("output.txt");

  if(shouldMakeManeuver) {
    output << valid_xd << ' ' << valid_yd << ' ';
  }

  float ratio = (valid_D - h) / valid_D;

  float fireX = valid_xd + (targetX - valid_xd) * ratio;
  float fireY = valid_yd + (targetY - valid_yd) * ratio;

 output << fireX << ' ' << fireY << std::endl;
 output.close();

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