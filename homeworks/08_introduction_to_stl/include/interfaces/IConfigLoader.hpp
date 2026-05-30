#pragma once
#include "Types.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class IConfigLoader {
public:
  virtual bool load(const std::string& droneConfigPath, const std::string& bombParamsPath) = 0;
  virtual DroneConfig getConfig() = 0;
  virtual BombParams getAmmoParams() = 0;
  virtual ~IConfigLoader() = default;
};