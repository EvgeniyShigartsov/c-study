#pragma once
#include "../interfaces/IConfigLoader.hpp"
#include "../interfaces/IBallisticSolver.hpp"
#include "../interfaces/ITargetProvider.hpp"

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

IConfigLoader* createLoader(LoaderType type);
IBallisticSolver* createSolver(SolverType type);
ITargetProvider* createProvider(ProviderType type, const char* param, const DroneConfig& droneConfig);