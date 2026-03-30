#include <SDL3/SDL.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "sim/SceneCsv.h"
#include "sim/SceneSetup.h"
#include "sim/Scenario.h"
#include "sim/Simulation.h"

namespace {

struct Options {
    std::string mode = "visual";
    std::string scenario = "container";
    int steps = 1800;
    int dumpEvery = 120;
    int dumpBalls = 0;
    int balls = 1000;
    std::uint32_t seed = 7;
    double restitution = 0.25;
    double gravity = 120.0;
    double dt = 1.0 / 60.0;
    int sceneWidth = 720;
    int sceneHeight = 720;
    double radius = 6.0;
    double linearDamping = 40.0;
    double sleepBounceSpeed = 12.0;
    double allowedTravelPerSubstep = 0.35;
    double overlapSlop = 0.0005;
    int solverIterations = 8;
    int maxSubsteps = 8;
    bool dumpFinal = false;
    std::string sceneCsvPath;
    std::string outputCsvPath;
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

        if (arg == "--mode") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.mode = *value;
        } else if (arg == "--scenario") {
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
        } else if (arg == "--dump-final") {
            options.dumpFinal = true;
        } else if (arg == "--scene-csv") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.sceneCsvPath = std::string(*value);
        } else if (arg == "--dump-final-csv") {
            const auto value = next(arg);
            if (!value) {
                return std::nullopt;
            }
            options.outputCsvPath = std::string(*value);
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            return std::nullopt;
        }
    }
    return options;
}

sim::Simulation makeSimulation(const Options& options) {
    sim::Scene scene;
    if (!options.sceneCsvPath.empty()) {
        scene = sim::makeInsetBoxScene(
            static_cast<double>(options.sceneWidth),
            static_cast<double>(options.sceneHeight),
            48.0,
            "csv");
        sim::loadSceneCsv(options.sceneCsvPath, scene);
    } else {
        sim::ScenarioOptions scenarioOptions;
        scenarioOptions.name = options.scenario;
        scenarioOptions.ballCount = options.balls;
        scenarioOptions.restitution = options.restitution;
        scenarioOptions.gravity = options.gravity;
        scenarioOptions.seed = options.seed;
        scenarioOptions.radius = options.radius;
        scenarioOptions.width = static_cast<double>(options.sceneWidth);
        scenarioOptions.height = static_cast<double>(options.sceneHeight);
        scene = sim::buildScenario(scenarioOptions);
    }

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

void maybeDumpFinalCsv(const Options& options, const sim::Simulation& simulation) {
    if (options.outputCsvPath.empty()) {
        return;
    }
    std::filesystem::path outputPath(options.outputCsvPath);
    if (outputPath.has_parent_path()) {
        std::filesystem::create_directories(outputPath.parent_path());
    }
    sim::saveSceneCsv(outputPath, simulation.scene());
}

void printStats(const sim::StepStats& stats) {
    std::cout << "frame=" << stats.frameIndex << " substeps=" << stats.substeps
              << " ball_ball=" << stats.ballBallContacts
              << " ball_wall=" << stats.ballWallContacts
              << " overlaps=" << stats.overlapCount
              << " max_penetration=" << stats.maxPenetration
              << " energy=" << stats.kineticEnergy
              << " max_speed=" << stats.maxSpeed
              << " escaped=" << stats.escapedBalls << '\n';
}

void printBallStates(const sim::Simulation& simulation, int count) {
    const auto& balls = simulation.scene().balls;
    const int limit = std::clamp(count, 0, static_cast<int>(balls.size()));
    for (int index = 0; index < limit; ++index) {
        const sim::Ball& ball = balls[static_cast<std::size_t>(index)];
        std::cout << "ball[" << index << "]"
                  << " pos=(" << ball.position.x << "," << ball.position.y << ")"
                  << " vel=(" << ball.velocity.x << "," << ball.velocity.y << ")\n";
    }
}

int runHeadless(const Options& options) {
    sim::Simulation simulation = makeSimulation(options);
    for (int step = 0; step < options.steps; ++step) {
        simulation.step();
        if (options.dumpEvery > 0 &&
            (step % options.dumpEvery == 0 || step + 1 == options.steps)) {
            printStats(simulation.lastStats());
        }
    }

    if (options.dumpFinal) {
        printStats(simulation.lastStats());
    }
    maybeDumpFinalCsv(options, simulation);
    if (options.dumpBalls > 0) {
        printBallStates(simulation, options.dumpBalls);
    }

    const sim::StepStats stats = simulation.lastStats();
    if (stats.escapedBalls != 0 || stats.maxPenetration > 0.01) {
        printStats(stats);
        return 1;
    }
    return 0;
}

void drawFilledCircle(SDL_Renderer* renderer, float cx, float cy, float radius) {
    const int minY = static_cast<int>(std::floor(cy - radius));
    const int maxY = static_cast<int>(std::ceil(cy + radius));
    const float radiusSq = radius * radius;

    for (int y = minY; y <= maxY; ++y) {
        const float dy = static_cast<float>(y) - cy;
        const float dxSq = radiusSq - dy * dy;
        if (dxSq < 0.0f) {
            continue;
        }
        const float dx = std::sqrt(dxSq);
        SDL_RenderLine(renderer, cx - dx, static_cast<float>(y), cx + dx, static_cast<float>(y));
    }
}

int runVisual(const Options& options) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    sim::Simulation simulation = makeSimulation(options);
    const sim::Scene& scene = simulation.scene();
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    if (!SDL_CreateWindowAndRenderer(
            "2D Physics Simulator",
            options.sceneWidth,
            options.sceneHeight,
            0,
            &window,
            &renderer)) {
        std::cerr << "SDL_CreateWindowAndRenderer failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    bool running = true;
    bool paused = false;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            } else if (event.type == SDL_EVENT_KEY_DOWN) {
                if (event.key.key == SDLK_ESCAPE) {
                    running = false;
                } else if (event.key.key == SDLK_SPACE) {
                    paused = !paused;
                }
            }
        }

        if (!paused) {
            simulation.step();
        }

        SDL_SetRenderDrawColor(renderer, 248, 246, 238, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 32, 39, 54, 255);
        SDL_FRect boundsRect{
            static_cast<float>(simulation.scene().bounds.minX),
            static_cast<float>(simulation.scene().bounds.minY),
            static_cast<float>(simulation.scene().bounds.maxX - simulation.scene().bounds.minX),
            static_cast<float>(simulation.scene().bounds.maxY - simulation.scene().bounds.minY)};
        SDL_RenderRect(renderer, &boundsRect);
        for (const sim::Wall& wall : simulation.scene().walls) {
            SDL_RenderLine(
                renderer,
                static_cast<float>(wall.a.x),
                static_cast<float>(wall.a.y),
                static_cast<float>(wall.b.x),
                static_cast<float>(wall.b.y));
        }

        for (const sim::Ball& ball : simulation.scene().balls) {
            SDL_SetRenderDrawColor(
                renderer,
                ball.color.r,
                ball.color.g,
                ball.color.b,
                ball.color.a);
            drawFilledCircle(
                renderer,
                static_cast<float>(ball.position.x),
                static_cast<float>(ball.position.y),
                static_cast<float>(ball.radius));
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(1);
    }

    maybeDumpFinalCsv(options, simulation);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

void printUsage() {
    std::cerr << "usage: simulator [--mode visual|headless] [--scenario container|stack|gap]"
                 " [--balls N] [--steps N] [--dump-every N] [--dump-balls N] [--dump-final]"
                 " [--seed N] [--restitution R] [--gravity G] [--dt DT]"
                 " [--scene-width N] [--scene-height N] [--radius R]"
                 " [--linear-damping D] [--sleep-bounce-speed V] [--allowed-travel X]"
                 " [--overlap-slop X] [--solver-iterations N] [--max-substeps N]"
                 " [--scene-csv PATH] [--dump-final-csv PATH]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::optional<Options> options = parseOptions(argc, argv);
        if (!options) {
            printUsage();
            return 2;
        }

        if (options->mode == "headless") {
            return runHeadless(*options);
        }
        if (options->mode == "visual") {
            return runVisual(*options);
        }

        printUsage();
        return 2;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
}
