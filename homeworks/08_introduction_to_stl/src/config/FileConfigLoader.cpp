#include <fstream>
#include <iostream>
#include "types.hpp"
#include "Logger.hpp"
#include "third_party/json.hpp"
#include "config//FileConfigLoader.hpp"

using json = nlohmann::json;

bool FileConfigLoader::readDroneConfig(const std::string& pathToConfig)
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

bool FileConfigLoader::readBombParams(const std::string& bombParamsPath)
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

bool FileConfigLoader::load(const std::string& pathToConfig, const std::string& bombParamsPath)
{
  const bool isConfigLoadOk = readDroneConfig(pathToConfig);
  const bool isBombParamsLoadOk = readBombParams(bombParamsPath);

  if (isConfigLoadOk && isBombParamsLoadOk) {
    LOG("Config loaded: speed=" << droneConfig.v0);
    LOG("Ammo found: " << bombParams.name);
  }

  return isConfigLoadOk && isBombParamsLoadOk;
}

DroneConfig FileConfigLoader::getConfig()
{
  return droneConfig;
}

BombParams FileConfigLoader::getAmmoParams()
{
  return bombParams;
}

FileConfigLoader::~FileConfigLoader() = default;