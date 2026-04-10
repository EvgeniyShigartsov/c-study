#include <cstring>
#include <iostream>
#include <fstream>

float get_h(float t, float d, float g, float l, float m, float v0){

    float term1 =
        (pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (pow(l, 2) - 1) * v0))
        /
        (36 * pow(m, 2));

    float term2 = 
     (pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * pow(l, 2) * (pow(l, 2) + 1) * v0))
     /
     (36 * (pow(l, 2) + 1) * pow(m, 4));

     float term3 = 
     (pow(t, 4) *
        (3 * pow(d, 3) * (pow(l, 2) + 1) * pow(l, 2) * v0 + 6 * pow(d, 3) * (pow(l, 2) + 1) * pow(l, 4) * v0 
        - 6 * pow(d, 2) * g * (pow(l, 4) + pow(l, 2) + 1) * l * m)
    )
     /
     (36 * pow(pow(l, 2) + 1, 2) * pow(m, 3));

     float term4 = (d * pow(t, 2) * v0) / (m * 2);


    return term1 + term2 + term3 - term4 + (t * v0);
}

int main(){
  std::ifstream file("input.txt");
  std::float_t xd;
  std::float_t yd;
  std::float_t zd;
  std::float_t targetX;
  std::float_t targetY;
  std::float_t v0;
  std::float_t accelerationPath;
  
  char ammo_name[12];

  file >> xd >> yd >> zd >> targetX >> targetY >> v0 >> accelerationPath >> ammo_name;
  file.close();

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
    return 1;
}

  float a = (d * g * m) - ((pow(d, 2) * 2) * l * v0);
  float b = ((-3 * g) * (pow(m, 2))) + ((d * 3) * l * m * v0);
  float c = (6 * pow(m, 2)) * zd;

  float p = -pow(b, 2) / (3 * pow(a, 2));
  float q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

  float angle = 3 * q / (2 * p) * sqrt(-3 / p);

  if(angle > 1.0f || angle < -1.0f){
    return 1;
  }

  float fi = acos(angle);
  float t = 2 * sqrt((-p / 3) * cos((fi + M_PI * 4) / 3) - b / 3 * a);

  float h = get_h(t, d, g, l, m, v0);

  float D = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));

  bool shouldMakeManeuver = h + accelerationPath > D;

  float valid_xd = shouldMakeManeuver ? targetX - (targetX - xd) * (h + accelerationPath) / D : xd;
  float valid_yd = shouldMakeManeuver ? targetY - (targetY - yd) * (h + accelerationPath) / D : yd;

  float ratio = (D - h) / D;

  float fireX = valid_xd + (targetX - valid_xd) * ratio;
  float fireY = valid_yd + (targetY - valid_yd) * ratio;


  std::cout << m << ' ' << d << ' ' <<  l << std::endl;

  return 0;
}