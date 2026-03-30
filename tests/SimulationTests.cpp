#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

#include "sim/SceneCsv.h"
#include "sim/SceneSetup.h"
#include "sim/Scenario.h"
#include "sim/Simulation.h"

namespace {

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

sim::Simulation makeScenarioSimulation(const std::string& name,
                                       int ballCount,
                                       std::uint32_t seed,
                                       double dt) {
    sim::ScenarioOptions options;
    options.name = name;
    options.ballCount = ballCount;
    options.seed = seed;
    options.radius = 6.0;

    sim::SimulationConfig config;
    config.gravity = 1400.0;
    config.restitution = 0.25;
    config.fixedDt = dt;
    config.linearDamping = 0.05;
    config.sleepBounceSpeed = 32.0;
    config.sleepLinearSpeed = 32.0;
    config.allowedTravelPerSubstep = 0.25;
    config.overlapSlop = 0.001;
    config.maxLinearSpeed = 220.0;
    config.solverIterations = 8;
    config.maxSubsteps = 16;
    return sim::Simulation(sim::buildScenario(options), config);
}

bool testDeterministicReplay() {
    sim::Simulation a = makeScenarioSimulation("container", 120, 4444, 1.0 / 120.0);
    sim::Simulation b = makeScenarioSimulation("container", 120, 4444, 1.0 / 120.0);

    a.stepMany(240);
    b.stepMany(240);

    if (!expect(a.scene().balls.size() == b.scene().balls.size(), "ball counts diverged")) {
        return false;
    }

    for (std::size_t i = 0; i < a.scene().balls.size(); ++i) {
        const sim::Ball& left = a.scene().balls[i];
        const sim::Ball& right = b.scene().balls[i];
        if (!expectNear(left.position.x, right.position.x, 1e-10, "position x mismatch") ||
            !expectNear(left.position.y, right.position.y, 1e-10, "position y mismatch") ||
            !expectNear(left.velocity.x, right.velocity.x, 1e-10, "velocity x mismatch") ||
            !expectNear(left.velocity.y, right.velocity.y, 1e-10, "velocity y mismatch")) {
            return false;
        }
    }

    return true;
}

bool testContainerIntegrity() {
    sim::Simulation simulation = makeScenarioSimulation("container", 250, 7, 1.0 / 120.0);
    for (int step = 0; step < 300; ++step) {
        simulation.step();
        if (!expect(simulation.lastStats().escapedBalls == 0, "container lost balls")) {
            return false;
        }
    }
    return true;
}

bool testFastWallContainment() {
    sim::Scene scene = sim::makeBoxScene(420.0, 320.0, "fast_wall");
    scene.walls.push_back({{220.0, 20.0}, {220.0, 300.0}});

    sim::Ball ball;
    ball.position = {96.0, 120.0};
    ball.velocity = {520.0, 65.0};
    ball.radius = 10.0;
    ball.inverseMass = 1.0 / 100.0;
    scene.balls.push_back(ball);

    sim::SimulationConfig config;
    config.gravity = 0.0;
    config.restitution = 0.35;
    config.fixedDt = 1.0 / 30.0;
    config.linearDamping = 0.0;
    config.sleepBounceSpeed = 4.0;
    config.sleepLinearSpeed = 4.0;
    config.allowedTravelPerSubstep = 0.25;
    config.overlapSlop = 0.001;
    config.maxLinearSpeed = 600.0;
    config.solverIterations = 8;
    config.maxSubsteps = 24;

    sim::Simulation simulation(std::move(scene), config);
    for (int step = 0; step < 180; ++step) {
        simulation.step();
        const sim::Ball& currentBall = simulation.scene().balls.front();
        if (!expect(simulation.lastStats().escapedBalls == 0, "fast wall scene escaped bounds") ||
            !expect(currentBall.position.x <= 220.0 - currentBall.radius + 0.5,
                    "fast moving ball crossed the interior wall")) {
            return false;
        }
    }
    return true;
}

bool testCsvRoundTrip() {
    sim::Scene scene = sim::makeBoxScene(200.0, 160.0, "csv");

    sim::Ball first;
    first.position = {12.5, 20.25};
    first.radius = 6.0;
    first.inverseMass = 1.0 / 36.0;
    first.color = {12, 34, 56, 255};
    scene.balls.push_back(first);

    sim::Ball second;
    second.position = {42.75, 88.5};
    second.radius = 9.0;
    second.inverseMass = 1.0 / 81.0;
    second.color = {200, 150, 100, 255};
    scene.balls.push_back(second);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "simulate_u1406330_scene.csv";
    sim::saveSceneCsv(path, scene);

    sim::Scene loaded = sim::makeBoxScene(200.0, 160.0, "loaded");
    sim::loadSceneCsv(path, loaded);
    std::filesystem::remove(path);

    if (!expect(loaded.balls.size() == scene.balls.size(), "csv roundtrip changed ball count")) {
        return false;
    }

    for (std::size_t i = 0; i < scene.balls.size(); ++i) {
        const sim::Ball& expected = scene.balls[i];
        const sim::Ball& actual = loaded.balls[i];
        if (!expectNear(actual.position.x, expected.position.x, 1e-12, "csv x mismatch") ||
            !expectNear(actual.position.y, expected.position.y, 1e-12, "csv y mismatch") ||
            !expectNear(actual.radius, expected.radius, 1e-12, "csv radius mismatch") ||
            !expect(actual.color.r == expected.color.r &&
                        actual.color.g == expected.color.g &&
                        actual.color.b == expected.color.b,
                    "csv color mismatch")) {
            return false;
        }
    }

    return true;
}

bool testCsvPreservesWalls() {
    sim::Scene scene = sim::makeBoxScene(320.0, 240.0, "walls");
    scene.walls.push_back({{160.0, 60.0}, {160.0, 200.0}});
    const std::size_t wallCount = scene.walls.size();

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "simulate_u1406330_minimal_scene.csv";
    {
        std::ofstream out(path);
        out << "x,y,r,g,b,radius\n";
        out << "20,30,255,0,0,5\n";
        out << "60,80,0,255,0,7\n";
    }

    sim::loadSceneCsv(path, scene);
    std::filesystem::remove(path);

    if (!expect(scene.walls.size() == wallCount, "loading CSV should not replace walls")) {
        return false;
    }
    return expect(scene.balls.size() == 2, "loading CSV should replace balls");
}

using TestFn = bool (*)();

struct TestCase {
    std::string_view name;
    TestFn fn;
};

constexpr TestCase kTests[] = {
    {"deterministic_replay", &testDeterministicReplay},
    {"container_integrity", &testContainerIntegrity},
    {"fast_wall_containment", &testFastWallContainment},
    {"csv_roundtrip", &testCsvRoundTrip},
    {"csv_preserves_walls", &testCsvPreservesWalls},
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
