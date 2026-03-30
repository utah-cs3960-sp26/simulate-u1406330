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
    config.gravity = 120.0;
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

bool testPackedSceneHasHighDensity() {
    sim::ScenarioOptions options;
    options.name = "packed";
    options.ballCount = 0;
    options.width = 720.0;
    options.height = 720.0;
    options.radius = 6.0;

    const sim::Scene scene = sim::buildScenario(options);
    return expect(scene.balls.size() > 2000, "packed scene should fill most of the box with balls");
}

bool testPackedSceneStartsAsLaunchBlob() {
    sim::ScenarioOptions options;
    options.name = "packed";
    options.ballCount = 2300;
    options.width = 720.0;
    options.height = 720.0;
    options.radius = 6.0;

    const sim::Scene scene = sim::buildScenario(options);
    if (!expect(!scene.balls.empty(), "packed scene should contain balls")) {
        return false;
    }

    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double totalVx = 0.0;
    for (const sim::Ball& ball : scene.balls) {
        minX = std::min(minX, ball.position.x);
        maxX = std::max(maxX, ball.position.x);
        totalVx += ball.velocity.x;
    }

    const double occupiedWidth = maxX - minX;
    const double boxWidth = scene.bounds.maxX - scene.bounds.minX;
    return expect(occupiedWidth < boxWidth * 0.7, "packed scene should start in a partial-width blob") &&
           expect(totalVx / static_cast<double>(scene.balls.size()) > 8.0,
                  "packed scene should launch balls into the box");
}

bool testPackedSceneHonorsRequestedBallCountByScalingRadius() {
    sim::ScenarioOptions options;
    options.name = "packed";
    options.ballCount = 5000;
    options.width = 720.0;
    options.height = 720.0;
    options.radius = 6.0;

    const sim::Scene scene = sim::buildScenario(options);
    if (!expect(scene.balls.size() == 5000, "packed scene should honor requested ball count")) {
        return false;
    }

    return expect(scene.balls.front().radius < 6.0,
                  "packed scene should shrink radius when higher counts are requested");
}

bool testContainerIsSquare() {
    sim::ScenarioOptions options;
    options.name = "container";
    options.ballCount = 10;
    options.width = 900.0;
    options.height = 720.0;
    const sim::Scene scene = sim::buildScenario(options);

    const double width = scene.bounds.maxX - scene.bounds.minX;
    const double height = scene.bounds.maxY - scene.bounds.minY;
    return expectNear(width, height, 1e-12, "container bounds should be square") &&
           expect(scene.walls.size() == 4, "container should only have square boundary walls");
}

bool testBallsSettleInBox() {
    sim::Scene scene = sim::makeBoxScene(320.0, 320.0, "settle");
    for (int i = 0; i < 20; ++i) {
        sim::Ball ball;
        ball.position = {40.0 + static_cast<double>(i / 5) * 14.0,
                         40.0 + static_cast<double>(i % 5) * 14.0};
        ball.radius = 6.0;
        ball.inverseMass = 1.0 / 36.0;
        scene.balls.push_back(ball);
    }

    sim::SimulationConfig config;
    config.gravity = 120.0;
    config.restitution = 0.2;
    config.fixedDt = 1.0 / 60.0;
    config.linearDamping = 0.05;
    config.sleepBounceSpeed = 32.0;
    config.sleepLinearSpeed = 32.0;
    config.allowedTravelPerSubstep = 0.25;
    config.overlapSlop = 0.001;
    config.maxLinearSpeed = 220.0;
    config.solverIterations = 8;
    config.maxSubsteps = 16;

    sim::Simulation simulation(std::move(scene), config);
    int quietFrames = 0;
    for (int step = 0; step < 2400; ++step) {
        simulation.step();
        const sim::StepStats stats = simulation.lastStats();
        if (stats.maxSpeed <= 4.0 && stats.kineticEnergy <= 20.0) {
            ++quietFrames;
            if (quietFrames >= 120) {
                return true;
            }
        } else {
            quietFrames = 0;
        }
    }

    return expect(false, "balls never reached a sustained settled state");
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
    first.velocity = {1.5, -0.75};
    first.previousPosition = first.position - first.velocity;
    first.radius = 6.0;
    first.inverseMass = 1.0 / 36.0;
    first.color = {12, 34, 56, 255};
    scene.balls.push_back(first);

    sim::Ball second;
    second.position = {42.75, 88.5};
    second.velocity = {-2.25, 0.5};
    second.previousPosition = second.position - second.velocity;
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
            !expectNear(actual.previousPosition.x, expected.previousPosition.x, 1e-12,
                        "csv previous x mismatch") ||
            !expectNear(actual.previousPosition.y, expected.previousPosition.y, 1e-12,
                        "csv previous y mismatch") ||
            !expectNear(actual.velocity.x, expected.velocity.x, 1e-12, "csv vx mismatch") ||
            !expectNear(actual.velocity.y, expected.velocity.y, 1e-12, "csv vy mismatch") ||
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
        out << "x,y,previous_x,previous_y,vx,vy,r,g,b,radius\n";
        out << "20,30,18.75,30.5,1.25,-0.5,255,0,0,5\n";
        out << "60,80,62,79.25,-2.0,0.75,0,255,0,7\n";
    }

    sim::loadSceneCsv(path, scene);
    std::filesystem::remove(path);

    if (!expect(scene.walls.size() == wallCount, "loading CSV should not replace walls")) {
        return false;
    }
    return expect(scene.balls.size() == 2, "loading CSV should replace balls");
}

bool testCsvRoundTripPreservesSceneMetadata() {
    sim::Scene scene = sim::makeInsetBoxScene(640.0, 480.0, 40.0, "metadata");
    scene.walls.push_back({{180.0, 120.0}, {460.0, 120.0}});

    sim::Ball ball;
    ball.position = {96.0, 88.0};
    ball.radius = 7.5;
    ball.inverseMass = 1.0 / (ball.radius * ball.radius);
    ball.color = {10, 20, 30, 255};
    scene.balls.push_back(ball);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "simulate_u1406330_scene_metadata.csv";
    sim::saveSceneCsv(path, scene);

    sim::Scene loaded;
    sim::loadSceneCsv(path, loaded);
    std::filesystem::remove(path);

    if (!expectNear(loaded.bounds.minX, scene.bounds.minX, 1e-12, "bounds minX mismatch") ||
        !expectNear(loaded.bounds.minY, scene.bounds.minY, 1e-12, "bounds minY mismatch") ||
        !expectNear(loaded.bounds.maxX, scene.bounds.maxX, 1e-12, "bounds maxX mismatch") ||
        !expectNear(loaded.bounds.maxY, scene.bounds.maxY, 1e-12, "bounds maxY mismatch")) {
        return false;
    }

    if (!expect(loaded.walls.size() == scene.walls.size(), "wall count mismatch") ||
        !expect(loaded.balls.size() == scene.balls.size(), "ball count mismatch")) {
        return false;
    }

    for (std::size_t i = 0; i < scene.walls.size(); ++i) {
        if (!expectNear(loaded.walls[i].a.x, scene.walls[i].a.x, 1e-12, "wall ax mismatch") ||
            !expectNear(loaded.walls[i].a.y, scene.walls[i].a.y, 1e-12, "wall ay mismatch") ||
            !expectNear(loaded.walls[i].b.x, scene.walls[i].b.x, 1e-12, "wall bx mismatch") ||
            !expectNear(loaded.walls[i].b.y, scene.walls[i].b.y, 1e-12, "wall by mismatch")) {
            return false;
        }
    }

    return true;
}

using TestFn = bool (*)();

struct TestCase {
    std::string_view name;
    TestFn fn;
};

constexpr TestCase kTests[] = {
    {"deterministic_replay", &testDeterministicReplay},
    {"container_integrity", &testContainerIntegrity},
    {"packed_scene_has_high_density", &testPackedSceneHasHighDensity},
    {"packed_scene_starts_as_launch_blob", &testPackedSceneStartsAsLaunchBlob},
    {"packed_scene_honors_requested_ball_count", &testPackedSceneHonorsRequestedBallCountByScalingRadius},
    {"container_is_square", &testContainerIsSquare},
    {"balls_settle_in_box", &testBallsSettleInBox},
    {"fast_wall_containment", &testFastWallContainment},
    {"csv_roundtrip", &testCsvRoundTrip},
    {"csv_preserves_walls", &testCsvPreservesWalls},
    {"csv_roundtrip_preserves_scene_metadata", &testCsvRoundTripPreservesSceneMetadata},
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
