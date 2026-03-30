#include "sim/Scenario.h"
#include "sim/SceneSetup.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace sim {

namespace {

ColorRGBA makeRainbowColor(double t) {
    const auto channel = [&](double phase) -> std::uint8_t {
        const double value = 0.5 + 0.5 * std::sin((t + phase) * 6.28318530717958647692);
        return static_cast<std::uint8_t>(std::clamp(std::lround(value * 255.0), 0l, 255l));
    };
    return {channel(0.0), channel(0.3333333333333333), channel(0.6666666666666666), 255};
}

Ball makeBall(const Vec2& position, const Vec2& velocity, double radius, const ColorRGBA& color) {
    Ball ball;
    ball.position = position;
    ball.previousPosition = position;
    ball.velocity = velocity;
    ball.radius = radius;
    ball.inverseMass = 1.0 / std::max(1.0, radius * radius);
    ball.color = color;
    return ball;
}

void addContainerBalls(Scene& scene, const ScenarioOptions& options) {
    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<double> jitter(-0.08 * options.radius, 0.08 * options.radius);
    std::uniform_real_distribution<double> startVx(2.0, 8.0);
    std::uniform_real_distribution<double> startVy(-2.0, 2.0);

    const double side = scene.bounds.maxX - scene.bounds.minX;
    const double originX = scene.bounds.minX;
    const double originY = scene.bounds.minY;
    const double spacing = options.radius * 2.15;
    const double spawnX = options.radius * 2.2;
    const double startY = options.radius * 2.5;
    const double maxY = side * 0.72;
    const int rows =
        std::max(1, static_cast<int>(std::floor((maxY - startY) / std::max(1.0, spacing))));

    for (int index = 0; index < options.ballCount; ++index) {
        const int row = index % rows;
        const int column = index / rows;
        const double x = originX + spawnX + static_cast<double>(column) * spacing + jitter(rng);
        const double y = originY + startY + static_cast<double>(row) * spacing + jitter(rng);
        if (x + options.radius >= originX + side * 0.45 || y + options.radius >= originY + maxY) {
            break;
        }
        scene.balls.push_back(makeBall(
            {x, y},
            {startVx(rng), startVy(rng)},
            options.radius,
            makeRainbowColor(static_cast<double>(index) * 0.017 + static_cast<double>(options.seed) * 0.0001)));
    }
}

void addPackedBalls(Scene& scene, const ScenarioOptions& options) {
    const double boxWidth = scene.bounds.maxX - scene.bounds.minX;
    const double boxHeight = scene.bounds.maxY - scene.bounds.minY;
    int requestedCount = options.ballCount;
    const double radius = options.radius;
    if (requestedCount <= 0) {
        const double targetCoverage = 0.85;
        const double boxArea = boxWidth * boxHeight;
        const double ballArea = 3.14159265358979323846 * radius * radius;
        requestedCount = static_cast<int>(std::floor(targetCoverage * boxArea / std::max(ballArea, 1.0)));
    }

    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<double> jitter(-0.035 * radius, 0.035 * radius);
    std::uniform_real_distribution<double> startVx(-2.5, 2.5);
    std::uniform_real_distribution<double> startVy(2.0, 8.0);

    const double nozzleWidth = boxWidth * 0.42;
    const double nozzleCenterX = scene.bounds.minX + boxWidth * 0.5;
    const double nozzleMinX = nozzleCenterX - nozzleWidth * 0.5 + radius * 0.5;
    const double nozzleMaxX = nozzleCenterX + nozzleWidth * 0.5 - radius * 0.5;
    const double topClearance = boxHeight * 0.16;
    const double nozzleMinY = scene.bounds.minY - topClearance;
    const double nozzleMaxY = scene.bounds.minY - radius * 1.2;
    const double nozzleSpacingX = radius * 2.05;
    const double nozzleSpacingY = radius * 2.05;

    std::vector<Vec2> spawnPoints;
    for (double y = nozzleMinY; y <= nozzleMaxY; y += nozzleSpacingY) {
        for (double x = nozzleMinX; x <= nozzleMaxX; x += nozzleSpacingX) {
            spawnPoints.push_back({x, y});
        }
    }
    if (spawnPoints.empty()) {
        spawnPoints.push_back({scene.bounds.minX + radius * 1.5, scene.bounds.minY + boxHeight * 0.5});
    }

    const int spawnBatchSize = 4;
    for (int index = 0; index < requestedCount; ++index) {
        const Vec2 base = spawnPoints[static_cast<std::size_t>(index % static_cast<int>(spawnPoints.size()))];
        const double verticalBias =
            (base.y - nozzleMinY) / std::max(nozzleMaxY - nozzleMinY, 1.0);
        scene.balls.push_back(makeBall(
            {base.x + jitter(rng), base.y + jitter(rng)},
            {startVx(rng), startVy(rng) + (verticalBias - 0.5) * 2.0},
            radius,
            makeRainbowColor(static_cast<double>(index) * 0.009 +
                             static_cast<double>(options.seed) * 0.0001)));
        scene.balls.back().spawnFrame = index / spawnBatchSize;
        scene.balls.back().emitted = scene.balls.back().spawnFrame <= 0;
    }
}

Scene buildContainerScene(const ScenarioOptions& options) {
    const double margin =
        std::max(24.0, std::min(options.width, options.height) * 0.08);
    const double side =
        std::max(240.0, std::min(options.width, options.height) - margin * 2.0);
    const Vec2 offset{
        (options.width - side) * 0.5,
        (options.height - side) * 0.5};
    Scene scene;
    scene.name = "container";
    scene.bounds = {offset.x, offset.y, offset.x + side, offset.y + side};
    scene.walls.push_back({{scene.bounds.maxX, scene.bounds.minY}, {scene.bounds.maxX, scene.bounds.maxY}});
    scene.walls.push_back({{scene.bounds.maxX, scene.bounds.maxY}, {scene.bounds.minX, scene.bounds.maxY}});
    scene.walls.push_back({{scene.bounds.minX, scene.bounds.maxY}, {scene.bounds.minX, scene.bounds.minY}});
    addContainerBalls(scene, options);
    return scene;
}

Scene buildStackScene(const ScenarioOptions& options) {
    Scene scene;
    scene.name = "stack";
    scene.bounds = {0.0, 0.0, 320.0, 520.0};
    addBoundaryWalls(scene, scene.bounds.maxX, scene.bounds.maxY);

    const double radius = options.radius;
    for (int i = 0; i < 10; ++i) {
        const double y = 70.0 + static_cast<double>(i) * radius * 2.12;
        scene.balls.push_back(
            makeBall({160.0, y}, {0.0, 0.0}, radius, makeRainbowColor(static_cast<double>(i) * 0.11)));
    }
    return scene;
}

Scene buildPackedScene(const ScenarioOptions& options) {
    const double margin =
        std::max(24.0, std::min(options.width, options.height) * 0.08);
    const double side =
        std::max(240.0, std::min(options.width, options.height) - margin * 2.0);
    const Vec2 offset{
        (options.width - side) * 0.5,
        (options.height - side) * 0.5};
    Scene scene;
    scene.name = "packed";
    scene.bounds = {offset.x, offset.y, offset.x + side, offset.y + side};

    scene.walls.push_back({{scene.bounds.maxX, scene.bounds.minY}, {scene.bounds.maxX, scene.bounds.maxY}});
    scene.walls.push_back({{scene.bounds.maxX, scene.bounds.maxY}, {scene.bounds.minX, scene.bounds.maxY}});
    scene.walls.push_back({{scene.bounds.minX, scene.bounds.maxY}, {scene.bounds.minX, scene.bounds.minY}});

    addPackedBalls(scene, options);
    return scene;
}

Scene buildGapScene(const ScenarioOptions& options) {
    Scene scene;
    scene.name = "gap";
    scene.bounds = {0.0, 0.0, 640.0, 420.0};
    addBoundaryWalls(scene, scene.bounds.maxX, scene.bounds.maxY);

    const double slitWidth = options.radius * 1.6;
    const double midY = 240.0;
    const double centerX = 320.0;
    scene.walls.push_back({{0.0, midY}, {centerX - slitWidth * 0.5, midY}});
    scene.walls.push_back({{centerX + slitWidth * 0.5, midY}, {640.0, midY}});

    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<double> jitter(-0.03 * options.radius, 0.03 * options.radius);
    std::uniform_real_distribution<double> vx(8.0, 16.0);
    const double spacingX = options.radius * 2.22;
    const double spacingY = options.radius * 2.18;
    const int columns = 10;
    for (int index = 0; index < options.ballCount; ++index) {
        const int row = index / columns;
        const int col = index % columns;
        const double x = 100.0 + static_cast<double>(col) * spacingX + jitter(rng);
        const double y = 60.0 + static_cast<double>(row) * spacingY + jitter(rng);
        if (y + options.radius >= midY - options.radius) {
            break;
        }
        scene.balls.push_back(makeBall(
            {x, y},
            {vx(rng), 0.0},
            options.radius,
            makeRainbowColor(static_cast<double>(index) * 0.021 + static_cast<double>(options.seed) * 0.0002)));
    }
    return scene;
}

}  // namespace

Scene buildScenario(const ScenarioOptions& options) {
    if (options.name == "container") {
        return buildContainerScene(options);
    }
    if (options.name == "stack") {
        return buildStackScene(options);
    }
    if (options.name == "packed") {
        return buildPackedScene(options);
    }
    if (options.name == "gap") {
        return buildGapScene(options);
    }
    throw std::invalid_argument("unknown scenario: " + options.name);
}

}  // namespace sim
