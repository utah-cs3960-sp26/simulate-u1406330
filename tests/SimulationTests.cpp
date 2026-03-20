#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string_view>
#include <vector>

#include "sim/Scenario.h"
#include "sim/Simulation.h"

namespace {

sim::Simulation makeSimulation(const std::string& scenarioName,
                               int ballCount,
                               std::uint32_t seed,
                               double restitution,
                               double dt,
                               double radius = 6.0) {
    sim::ScenarioOptions options;
    options.name = scenarioName;
    options.ballCount = ballCount;
    options.seed = seed;
    options.restitution = restitution;
    options.radius = radius;

    sim::SimulationConfig config;
    config.gravity = 1400.0;
    config.restitution = restitution;
    config.fixedDt = dt;
    config.linearDamping = 0.05;
    config.sleepBounceSpeed = 32.0;
    config.sleepLinearSpeed = 32.0;
    config.allowedTravelPerSubstep = 0.2;
    config.overlapSlop = 0.0005;
    config.maxLinearSpeed = 180.0;
    config.solverIterations = 10;
    config.maxSubsteps = 12;
    return sim::Simulation(sim::buildScenario(options), config);
}

sim::Simulation makeBoxSimulation(int ballCount,
                                  std::uint32_t seed,
                                  double restitution,
                                  double dt) {
    sim::Scene scene;
    scene.name = "box";
    scene.bounds = {0.0, 0.0, 360.0, 360.0};
    scene.walls.push_back({{0.0, 0.0}, {360.0, 0.0}});
    scene.walls.push_back({{360.0, 0.0}, {360.0, 360.0}});
    scene.walls.push_back({{360.0, 360.0}, {0.0, 360.0}});
    scene.walls.push_back({{0.0, 360.0}, {0.0, 0.0}});

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.05, 0.05);

    const double radius = 6.0;
    const double spacing = radius * 2.12;
    const int columns = 4;
    for (int index = 0; index < ballCount; ++index) {
        const int row = index / columns;
        const int column = index % columns;
        sim::Ball ball;
        ball.position = {
            120.0 + static_cast<double>(column) * spacing + jitter(rng),
            48.0 + static_cast<double>(row) * spacing + jitter(rng)};
        ball.velocity = {0.0, 0.0};
        ball.radius = radius;
        ball.inverseMass = 1.0 / (radius * radius);
        scene.balls.push_back(ball);
    }

    sim::SimulationConfig config;
    config.gravity = 1400.0;
    config.restitution = restitution;
    config.fixedDt = dt;
    config.linearDamping = 0.05;
    config.sleepBounceSpeed = 32.0;
    config.sleepLinearSpeed = 32.0;
    config.allowedTravelPerSubstep = 0.2;
    config.overlapSlop = 0.0005;
    config.maxLinearSpeed = 180.0;
    config.solverIterations = 10;
    config.maxSubsteps = 12;
    return sim::Simulation(std::move(scene), config);
}

bool expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool expectNear(double actual, double expected, double epsilon, std::string_view message) {
    if (std::abs(actual - expected) > epsilon) {
        std::cerr << "FAIL: " << message << " expected=" << expected << " actual=" << actual
                  << " epsilon=" << epsilon << '\n';
        return false;
    }
    return true;
}

bool testDeterministicReplay() {
    sim::Simulation a = makeSimulation("container", 160, 4444, 0.35, 1.0 / 120.0);
    sim::Simulation b = makeSimulation("container", 160, 4444, 0.35, 1.0 / 120.0);

    a.stepMany(480);
    b.stepMany(480);

    const auto& ballsA = a.scene().balls;
    const auto& ballsB = b.scene().balls;
    if (!expect(ballsA.size() == ballsB.size(), "ball counts diverged")) {
        return false;
    }

    for (std::size_t i = 0; i < ballsA.size(); ++i) {
        if (!expectNear(ballsA[i].position.x, ballsB[i].position.x, 1e-10, "position x mismatch") ||
            !expectNear(ballsA[i].position.y, ballsB[i].position.y, 1e-10, "position y mismatch") ||
            !expectNear(ballsA[i].velocity.x, ballsB[i].velocity.x, 1e-10, "velocity x mismatch") ||
            !expectNear(ballsA[i].velocity.y, ballsB[i].velocity.y, 1e-10, "velocity y mismatch")) {
            return false;
        }
    }

    return true;
}

bool testRestitutionChangesEnergyNotPacking() {
    sim::Simulation low = makeBoxSimulation(16, 1337, 0.05, 1.0 / 240.0);
    sim::Simulation high = makeBoxSimulation(16, 1337, 0.85, 1.0 / 240.0);

    auto firstQuietFrame = [](sim::Simulation& simulation) {
        int quietRun = 0;
        for (int i = 0; i < 2600; ++i) {
            simulation.step();
            const sim::StepStats stats = simulation.lastStats();
            if (stats.kineticEnergy < 1.0 && stats.maxSpeed < 8.0) {
                ++quietRun;
            } else {
                quietRun = 0;
            }
            if (quietRun >= 120) {
                return i;
            }
        }
        return 2600;
    };

    const int lowQuietFrame = firstQuietFrame(low);
    const int highQuietFrame = firstQuietFrame(high);

    if (!expect(lowQuietFrame <= highQuietFrame, "lower restitution did not settle faster")) {
        return false;
    }

    auto bounds = [](const sim::Simulation& simulation) {
        double minX = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();
        for (const sim::Ball& ball : simulation.scene().balls) {
            minX = std::min(minX, ball.position.x - ball.radius);
            maxX = std::max(maxX, ball.position.x + ball.radius);
            minY = std::min(minY, ball.position.y - ball.radius);
            maxY = std::max(maxY, ball.position.y + ball.radius);
        }
        return std::array<double, 4>{minX, maxX, minY, maxY};
    };

    const auto lowBounds = bounds(low);
    const auto highBounds = bounds(high);
    const double lowWidth = lowBounds[1] - lowBounds[0];
    const double highWidth = highBounds[1] - highBounds[0];
    const double lowHeight = lowBounds[3] - lowBounds[2];
    const double highHeight = highBounds[3] - highBounds[2];
    const double lowArea = lowWidth * lowHeight;
    const double highArea = highWidth * highHeight;

    if (!expectNear(lowArea, highArea, std::max(25.0, lowArea * 0.1), "settled packing area diverged")) {
        return false;
    }

    return expect(low.lastStats().kineticEnergy < 2.0, "low restitution stack did not settle");
}

bool testContainerIntegrity() {
    sim::Simulation simulation = makeSimulation("container", 1000, 7, 0.25, 1.0 / 60.0);
    simulation.stepMany(1800);
    const sim::StepStats stats = simulation.lastStats();
    return expect(stats.escapedBalls == 0, "container lost balls") &&
           expect(stats.maxPenetration < 0.01, "container penetration remained too high");
}

bool testGapIntegrity() {
    sim::Simulation simulation = makeSimulation("gap", 168, 2024, 0.18, 1.0 / 180.0);
    simulation.stepMany(1800);
    const sim::StepStats stats = simulation.lastStats();
    if (!expect(stats.escapedBalls == 0, "gap scenario escaped the outer box") ||
        !expect(stats.maxPenetration < 0.01, "gap scenario penetration remained too high")) {
        return false;
    }

    double maxY = 0.0;
    for (const sim::Ball& ball : simulation.scene().balls) {
        maxY = std::max(maxY, ball.position.y + ball.radius);
    }
    return expect(maxY <= 240.0 + 6.0 + 1.0, "ball squeezed through the tiny slit");
}

using TestFn = bool (*)();

struct TestCase {
    std::string_view name;
    TestFn fn;
};

constexpr TestCase kTests[] = {
    {"deterministic_replay", &testDeterministicReplay},
    {"restitution_energy_and_packing", &testRestitutionChangesEnergyNotPacking},
    {"container_integrity", &testContainerIntegrity},
    {"gap_integrity", &testGapIntegrity},
};

}  // namespace

int main(int argc, char** argv) {
    std::string_view requested = "all";
    if (argc > 1) {
        requested = argv[1];
    }

    bool ranAny = false;
    for (const TestCase& test : kTests) {
        if (requested != "all" && requested != test.name) {
            continue;
        }
        ranAny = true;
        std::cout << "running " << test.name << '\n';
        if (!test.fn()) {
            std::cerr << "test failed: " << test.name << '\n';
            return EXIT_FAILURE;
        }
    }

    if (!ranAny) {
        std::cerr << "unknown test: " << requested << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "ok\n";
    return EXIT_SUCCESS;
}
