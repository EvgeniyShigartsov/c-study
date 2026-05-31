#include "config/ComponentFactory.hpp"
#include <string>
#include "config/FileConfigLoader.hpp"
#include "types.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "solvers/AnalyticalSolver.hpp"

// NOLINTBEGIN(cppcoreguidelines-owning-memory)

IConfigLoader* createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return new FileConfigLoader;
    default:
      return nullptr;
  }
}

IBallisticSolver* createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return new AnalyticalSolver;
    default:
      return nullptr;
  }
}

ITargetProvider* createProvider(ProviderType type, const std::string& pathToConfig, const DroneConfig& droneConfig)
{
  switch (type) {
    case ProviderType::JSON:
      return new JsonTargetProvider{pathToConfig, droneConfig};
    default:
      return nullptr;
  }
}

// NOLINTEND(cppcoreguidelines-owning-memory)