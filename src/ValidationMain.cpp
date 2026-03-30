#include <algorithm>
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
    int steps = 1800;
    int dumpEvery = 120;
    int dumpBalls = 0;
    int balls = 1000;
    std::uint32_t seed = 7;
    double restitution = 0.25;
    double gravity = 120.0;
    double dt = 1.0 / 60.0;
    double radius = 6.0;
    double linearDamping = 40.0;
    double sleepBounceSpeed = 12.0;
    double allowedTravelPerSubstep = 0.35;
    double overlapSlop = 0.0005;
    int solverIterations = 8;
    int maxSubsteps = 8;
    int quietWindow = 0;
    double maxQuietEnergy = 1.0;
    double maxQuietSpeed = 1.0;
    double maxPenetration = 0.01;
    bool requireNoEscapeEveryFrame = false;
    bool dumpFinal = false;
};

bool parseInt(std::string_view text, int& value) {
    const std::string copy(text);
    char* end = nullptr;
    const long parsed = std::strtol(copy.c_str(), &end, 10);
    if (end == nullptr || *end != '\0') {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool parseUint(std::string_view text, std::uint32_t& value) {
    const std::string copy(text);
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(copy.c_str(), &end, 10);
    if (end == nullptr || *end != '\0') {
        return false;
    }
    value = static_cast<std::uint32_t>(parsed);
    return true;
}

bool parseDouble(std::string_view text, double& value) {
    const std::string copy(text);
    char* end = nullptr;
    value = std::strtod(copy.c_str(), &end);
    return end != nullptr && *end == '\0';
}

std::optional<Options> parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        auto next = [&](std::string_view name) -> std::optional<std::string_view> {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << '\n';
                return std::nullopt;
            }
            ++i;
            return argv[i];
        };

        if (arg == "--scenario") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.scenario = *value;
        } else if (arg == "--steps") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.steps)) {
                return std::nullopt;
            }
        } else if (arg == "--dump-every") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.dumpEvery)) {
                return std::nullopt;
            }
        } else if (arg == "--dump-balls") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.dumpBalls)) {
                return std::nullopt;
            }
        } else if (arg == "--balls") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.balls)) {
                return std::nullopt;
            }
        } else if (arg == "--seed") {
            const auto value = next(arg);
            if (!value || !parseUint(*value, options.seed)) {
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
        } else if (arg == "--radius") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.radius)) {
                return std::nullopt;
            }
        } else if (arg == "--linear-damping") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.linearDamping)) {
                return std::nullopt;
            }
        } else if (arg == "--sleep-bounce-speed") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.sleepBounceSpeed)) {
                return std::nullopt;
            }
        } else if (arg == "--allowed-travel") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.allowedTravelPerSubstep)) {
                return std::nullopt;
            }
        } else if (arg == "--overlap-slop") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.overlapSlop)) {
                return std::nullopt;
            }
        } else if (arg == "--solver-iterations") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.solverIterations)) {
                return std::nullopt;
            }
        } else if (arg == "--max-substeps") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.maxSubsteps)) {
                return std::nullopt;
            }
        } else if (arg == "--quiet-window") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.quietWindow)) {
                return std::nullopt;
            }
        } else if (arg == "--max-quiet-energy") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.maxQuietEnergy)) {
                return std::nullopt;
            }
        } else if (arg == "--max-quiet-speed") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.maxQuietSpeed)) {
                return std::nullopt;
            }
        } else if (arg == "--max-penetration") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.maxPenetration)) {
                return std::nullopt;
            }
        } else if (arg == "--require-no-escape-every-frame") {
            options.requireNoEscapeEveryFrame = true;
        } else if (arg == "--dump-final") {
            options.dumpFinal = true;
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            return std::nullopt;
        }
    }
    return options;
}

sim::Simulation makeSimulation(const Options& options) {
    sim::ScenarioOptions scenarioOptions;
    scenarioOptions.name = options.scenario;
    scenarioOptions.ballCount = options.balls;
    scenarioOptions.restitution = options.restitution;
    scenarioOptions.gravity = options.gravity;
    scenarioOptions.seed = options.seed;
    scenarioOptions.radius = options.radius;

    sim::SimulationConfig config;
    config.gravity = options.gravity;
    config.restitution = options.restitution;
    config.fixedDt = options.dt;
    config.linearDamping = options.linearDamping;
    config.sleepBounceSpeed = options.sleepBounceSpeed;
    config.allowedTravelPerSubstep = options.allowedTravelPerSubstep;
    config.overlapSlop = options.overlapSlop;
    config.solverIterations = options.solverIterations;
    config.maxSubsteps = options.maxSubsteps;
    return sim::Simulation(sim::buildScenario(scenarioOptions), config);
}

void printUsage() {
    std::cerr << "usage: physics_validation [--scenario container|stack|packed|gap] [--balls N]"
                 " [--steps N] [--dump-every N] [--dump-balls N] [--dump-final]"
                 " [--seed N] [--restitution R] [--gravity G] [--dt DT] [--radius R]"
                 " [--linear-damping D] [--sleep-bounce-speed V] [--allowed-travel X]"
                 " [--overlap-slop X] [--solver-iterations N] [--max-substeps N]"
                 " [--quiet-window N] [--max-quiet-energy E] [--max-quiet-speed V]"
                 " [--max-penetration X] [--require-no-escape-every-frame]\n";
}

void dumpBalls(const sim::Simulation& simulation, int count) {
    const auto& balls = simulation.scene().balls;
    const int limit = std::clamp(count, 0, static_cast<int>(balls.size()));
    for (int index = 0; index < limit; ++index) {
        const sim::Ball& ball = balls[static_cast<std::size_t>(index)];
        std::cout << "ball[" << index << "]"
                  << " pos=(" << ball.position.x << "," << ball.position.y << ")"
                  << " vel=(" << ball.velocity.x << "," << ball.velocity.y << ")\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::optional<Options> options = parseOptions(argc, argv);
        if (!options) {
            printUsage();
            return 2;
        }

        sim::Simulation simulation = makeSimulation(*options);
        int quietRun = 0;
        double worstPenetration = 0.0;
        double worstEnergy = 0.0;
        double worstSpeed = 0.0;
        int worstEscaped = 0;
        for (int step = 0; step < options->steps; ++step) {
            simulation.step();
            const sim::StepStats stats = simulation.lastStats();
            worstPenetration = std::max(worstPenetration, stats.maxPenetration);
            worstEnergy = std::max(worstEnergy, stats.kineticEnergy);
            worstSpeed = std::max(worstSpeed, stats.maxSpeed);
            worstEscaped = std::max(worstEscaped, stats.escapedBalls);
            if (options->quietWindow > 0 &&
                stats.kineticEnergy <= options->maxQuietEnergy &&
                stats.maxSpeed <= options->maxQuietSpeed) {
                ++quietRun;
            } else if (options->quietWindow > 0) {
                quietRun = 0;
            }
            if (options->dumpEvery > 0 &&
                (step % options->dumpEvery == 0 || step + 1 == options->steps)) {
                std::cout << "frame=" << stats.frameIndex << " substeps=" << stats.substeps
                          << " ball_ball=" << stats.ballBallContacts
                          << " ball_wall=" << stats.ballWallContacts
                          << " overlaps=" << stats.overlapCount
                          << " max_penetration=" << stats.maxPenetration
                          << " energy=" << stats.kineticEnergy
                          << " max_speed=" << stats.maxSpeed
                          << " escaped=" << stats.escapedBalls << '\n';
            }
        }

        const sim::StepStats stats = simulation.lastStats();
        if (options->dumpFinal) {
            std::cout << "frame=" << stats.frameIndex << " substeps=" << stats.substeps
                      << " ball_ball=" << stats.ballBallContacts
                      << " ball_wall=" << stats.ballWallContacts
                      << " overlaps=" << stats.overlapCount
                      << " max_penetration=" << stats.maxPenetration
                      << " energy=" << stats.kineticEnergy
                      << " max_speed=" << stats.maxSpeed
                      << " escaped=" << stats.escapedBalls << '\n';
        }
        if (options->dumpBalls > 0) {
            dumpBalls(simulation, options->dumpBalls);
        }
        if ((options->requireNoEscapeEveryFrame && worstEscaped != 0) ||
            stats.escapedBalls != 0 ||
            worstPenetration > options->maxPenetration ||
            (options->quietWindow > 0 && quietRun < options->quietWindow)) {
            std::cerr << "worst_penetration=" << worstPenetration
                      << " worst_energy=" << worstEnergy
                      << " worst_speed=" << worstSpeed
                      << " worst_escaped=" << worstEscaped
                      << " final_quiet_run=" << quietRun << '\n';
            return 1;
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
