#pragma once
#include "Types.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Target getTarget(const float simCurrentTime, const int targetIndex) = 0;
  virtual bool isLoadSucces() = 0;
  virtual ~ITargetProvider() = default;
};
