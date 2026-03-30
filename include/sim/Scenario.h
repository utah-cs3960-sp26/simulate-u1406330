#pragma once

#include <cstdint>
#include <string>

#include "sim/Simulation.h"

namespace sim {

struct ScenarioOptions {
    std::string name = "container";
    int ballCount = 1000;
    double restitution = 0.25;
    double gravity = 120.0;
    std::uint32_t seed = 7;
    double radius = 6.0;
    double width = 1280.0;
    double height = 720.0;
};

Scene buildScenario(const ScenarioOptions& options);

}  // namespace sim
