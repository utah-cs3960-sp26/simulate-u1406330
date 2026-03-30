#include <SDL3/SDL.h>

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "sim/SceneColorize.h"
#include "sim/SceneCsv.h"
#include "sim/SceneSetup.h"
#include "sim/Simulation.h"

namespace {

struct Options {
    std::string sceneCsvPath;
    std::string imagePath;
    std::string outputCsvPath;
    std::string finalCsvPath;
    int sceneWidth = 720;
    int sceneHeight = 720;
    double containerInset = 48.0;
    int steps = 1800;
    double restitution = 0.25;
    double gravity = 120.0;
    double dt = 1.0 / 60.0;
    double linearDamping = 40.0;
    double sleepBounceSpeed = 12.0;
    double allowedTravelPerSubstep = 0.35;
    double overlapSlop = 0.0005;
    int solverIterations = 8;
    int maxSubsteps = 8;
    int quietWindow = 120;
    double maxQuietSpeed = 8.0;
    double maxQuietEnergy = 100.0;
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

        if (arg == "--scene-csv") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.sceneCsvPath = std::string(*value);
        } else if (arg == "--image") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.imagePath = std::string(*value);
        } else if (arg == "--output-csv") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.outputCsvPath = std::string(*value);
        } else if (arg == "--final-csv") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.finalCsvPath = std::string(*value);
        } else if (arg == "--scene-width") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.sceneWidth)) {
                return std::nullopt;
            }
        } else if (arg == "--scene-height") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.sceneHeight)) {
                return std::nullopt;
            }
        } else if (arg == "--container-inset") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.containerInset)) {
                return std::nullopt;
            }
        } else if (arg == "--steps") {
            const auto value = next(arg);
            if (!value || !parseInt(*value, options.steps)) {
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
        } else if (arg == "--max-quiet-speed") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.maxQuietSpeed)) {
                return std::nullopt;
            }
        } else if (arg == "--max-quiet-energy") {
            const auto value = next(arg);
            if (!value || !parseDouble(*value, options.maxQuietEnergy)) {
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
    sim::Scene scene = sim::makeInsetBoxScene(
        static_cast<double>(options.sceneWidth),
        static_cast<double>(options.sceneHeight),
        options.containerInset,
        "image");
    sim::loadSceneCsv(options.sceneCsvPath, scene);

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
    return sim::Simulation(std::move(scene), config);
}

bool stepUntilSettled(sim::Simulation& simulation, const Options& options) {
    int quietFrames = 0;
    for (int step = 0; step < options.steps; ++step) {
        simulation.step();
        const sim::StepStats stats = simulation.lastStats();
        const bool quiet =
            stats.maxSpeed <= options.maxQuietSpeed &&
            stats.kineticEnergy <= options.maxQuietEnergy &&
            stats.overlapCount == 0;
        if (quiet) {
            ++quietFrames;
            if (quietFrames >= std::max(1, options.quietWindow)) {
                return true;
            }
        } else {
            quietFrames = 0;
        }
    }
    return false;
}

void printUsage() {
    std::cerr << "usage: scene_colorize --scene-csv PATH --image PATH --output-csv PATH"
                 " [--scene-width N] [--scene-height N] [--container-inset X]"
                 " [--steps N] [--restitution R] [--gravity G] [--dt DT]"
                 " [--linear-damping D] [--sleep-bounce-speed V]"
                 " [--allowed-travel X] [--overlap-slop X] [--solver-iterations N]"
                 " [--max-substeps N] [--quiet-window N]"
                 " [--max-quiet-speed V] [--max-quiet-energy E]"
                 " [--final-csv PATH]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::optional<Options> options = parseOptions(argc, argv);
        if (!options || options->sceneCsvPath.empty() || options->imagePath.empty() ||
            options->outputCsvPath.empty()) {
            printUsage();
            return 2;
        }

        std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
            SDL_LoadSurface(options->imagePath.c_str()),
            &SDL_DestroySurface);
        if (surface == nullptr) {
            std::cerr << "failed to load image: " << SDL_GetError() << '\n';
            return 1;
        }

        sim::Simulation simulation = makeSimulation(*options);
        sim::Scene initialScene = simulation.scene();
        if (!stepUntilSettled(simulation, *options)) {
            std::cerr << "failed to reach a settled scene within " << options->steps
                      << " steps\n";
            return 1;
        }
        const sim::Scene& settledScene = simulation.scene();
        const sim::Scene recoloredScene =
            sim::assignColorsFromSettledScene(initialScene, settledScene, surface.get());

        std::filesystem::path outputPath(options->outputCsvPath);
        if (outputPath.has_parent_path()) {
            std::filesystem::create_directories(outputPath.parent_path());
        }
        sim::saveSceneCsv(outputPath, recoloredScene);

        if (!options->finalCsvPath.empty()) {
            std::filesystem::path finalPath(options->finalCsvPath);
            if (finalPath.has_parent_path()) {
                std::filesystem::create_directories(finalPath.parent_path());
            }
            sim::saveSceneCsv(finalPath, settledScene);
        }

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
