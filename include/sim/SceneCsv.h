#pragma once

#include <filesystem>

#include "sim/Simulation.h"

namespace sim {

void loadSceneCsv(const std::filesystem::path& path, Scene& scene);
void saveSceneCsv(const std::filesystem::path& path, const Scene& scene);

}  // namespace sim
