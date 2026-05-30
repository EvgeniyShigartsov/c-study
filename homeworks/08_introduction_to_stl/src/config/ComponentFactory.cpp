#include "config/ComponentFactory.hpp"
#include "config/FileConfigLoader.hpp"
#include "Types.hpp"
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

ITargetProvider* createProvider(ProviderType type, const char* param, const DroneConfig& droneConfig)
{
  switch (type) {
    case ProviderType::JSON:
      return new JsonTargetProvider{param, droneConfig};
    default:
      return nullptr;
  }
}

// NOLINTEND(cppcoreguidelines-owning-memory)