#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include "sim/Scenario.h"
#include "sim/Simulation.h"

namespace {

struct Options {
    std::string scenario = "container";
    int balls = 1000;
    std::uint32_t seed = 7;
    int steps = 1200;
    int dumpEvery = 120;
    double restitution = 0.25;
    double gravity = 1400.0;
    double dt = 1.0 / 120.0;
};

template <typename T>
bool parseNumber(std::string_view text, T& value) {
    const char* begin = text.data();
    const char* end = text.data() + text.size();
    auto result = std::from_chars(begin, end, value);
    return result.ec == std::errc{} && result.ptr == end;
}

bool parseDouble(std::string_view text, double& value) {
    std::string owned(text);
    char* end = nullptr;
    value = std::strtod(owned.c_str(), &end);
    return end != nullptr && *end == '\0';
}

std::optional<Options> parseArgs(int argc, char** argv) {
    Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        auto next = [&](std::string_view name) -> std::optional<std::string_view> {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << '\n';
                return std::nullopt;
            }
            ++i;
            return std::string_view(argv[i]);
        };

        if (arg == "--scenario") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.scenario = std::string(*value);
        } else if (arg == "--balls") {
            const auto value = next(arg);
            if (!value || !parseNumber(*value, options.balls)) {
                return std::nullopt;
            }
        } else if (arg == "--seed") {
            const auto value = next(arg);
            if (!value || !parseNumber(*value, options.seed)) {
                return std::nullopt;
            }
        } else if (arg == "--steps") {
            const auto value = next(arg);
            if (!value || !parseNumber(*value, options.steps)) {
                return std::nullopt;
            }
        } else if (arg == "--dump-every") {
            const auto value = next(arg);
            if (!value || !parseNumber(*value, options.dumpEvery)) {
                return std::nullopt;
            }
        } else if (arg == "--restitution") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.restitution)) {
                return std::nullopt;
            }
        } else if (arg == "--gravity") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.gravity)) {
                return std::nullopt;
            }
        } else if (arg == "--dt") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.dt)) {
                return std::nullopt;
            }
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            return std::nullopt;
        }
    }

    return options;
}

sim::Simulation makeSimulation(const Options& options) {
    sim::ScenarioOptions scenario;
    scenario.name = options.scenario;
    scenario.ballCount = options.balls;
    scenario.seed = options.seed;
    scenario.gravity = options.gravity;
    scenario.restitution = options.restitution;

    sim::SimulationConfig config;
    config.gravity = options.gravity;
    config.restitution = options.restitution;
    config.fixedDt = options.dt;
    config.solverIterations = 8;
    config.maxSubsteps = 12;
    config.allowedTravelPerSubstep = 0.45;
    config.linearDamping = 0.015;
    config.sleepBounceSpeed = 18.0;

    return sim::Simulation(sim::buildScenario(scenario), config);
}

void dumpStats(const sim::StepStats& stats) {
    std::cout << "frame=" << stats.frameIndex
              << " substeps=" << stats.substeps
              << " ball_ball=" << stats.ballBallContacts
              << " ball_wall=" << stats.ballWallContacts
              << " overlaps=" << stats.overlapCount
              << " max_penetration=" << stats.maxPenetration
              << " energy=" << stats.kineticEnergy
              << " max_speed=" << stats.maxSpeed
              << " escaped=" << stats.escapedBalls
              << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    const auto options = parseArgs(argc, argv);
    if (!options) {
        std::cerr << "usage: physics_validation [--scenario container|stack|gap] [--balls N]"
                     " [--seed N] [--steps N] [--dump-every N] [--restitution R]"
                     " [--gravity G] [--dt SECONDS]\n";
        return 2;
    }

    try {
        sim::Simulation simulation = makeSimulation(*options);

        for (int step = 0; step < options->steps; ++step) {
            simulation.step();
            if (options->dumpEvery > 0 &&
                ((step % options->dumpEvery) == 0 || step + 1 == options->steps)) {
                dumpStats(simulation.lastStats());
            }
        }

        if (simulation.lastStats().escapedBalls != 0) {
            std::cerr << "validation failed: balls escaped the scene\n";
            return 1;
        }

        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
