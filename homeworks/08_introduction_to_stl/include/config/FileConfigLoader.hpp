#pragma once
#include "types.hpp"
#include "interfaces/IConfigLoader.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class FileConfigLoader : public IConfigLoader {
private:
  DroneConfig droneConfig{};
  BombParams bombParams{};

  bool readDroneConfig(const std::string& pathToConfig);
  bool readBombParams(const std::string& bombParamsPath);

public:
  bool load(const std::string& pathToConfig, const std::string& bombParamsPath) override;

  DroneConfig getConfig() override;
  BombParams getAmmoParams() override;
  virtual ~FileConfigLoader();
};