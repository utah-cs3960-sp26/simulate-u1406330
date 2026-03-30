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
    const double spawnWidth = boxWidth * 0.58;
    const double spawnHeight = boxHeight * 0.94;
    int requestedCount = options.ballCount;
    double radius = options.radius;
    if (requestedCount <= 0) {
        const double targetCoverage = 0.75;
        const double boxArea = boxWidth * boxHeight;
        const double ballArea = 3.14159265358979323846 * radius * radius;
        requestedCount = static_cast<int>(std::floor(targetCoverage * boxArea / std::max(ballArea, 1.0)));
    } else {
        const double targetCoverage = 0.82;
        const double spawnArea = spawnWidth * spawnHeight;
        const double fittedRadius = std::sqrt(
            (targetCoverage * spawnArea) /
            (static_cast<double>(requestedCount) * 3.14159265358979323846));
        radius = std::min(radius, std::max(2.0, fittedRadius));
    }

    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<double> jitter(-0.035 * radius, 0.035 * radius);
    std::uniform_real_distribution<double> startVx(10.0, 18.0);
    std::uniform_real_distribution<double> startVy(-6.0, 6.0);
    std::uniform_real_distribution<double> randomX(0.0, 1.0);
    std::uniform_real_distribution<double> randomY(0.0, 1.0);

    const double minX = scene.bounds.minX + radius * 1.1;
    const double maxX = minX + spawnWidth - radius * 1.1;
    const double minY = scene.bounds.minY + radius * 1.1;
    const double maxY = minY + spawnHeight - radius * 1.1;
    const double spacingX = radius * 2.02;
    const double spacingY = radius * std::sqrt(3.0) * 1.01;

    int index = 0;
    for (int row = 0; index < requestedCount; ++row) {
        const double y = minY + static_cast<double>(row) * spacingY;
        if (y > maxY) {
            break;
        }

        const double rowOffset = (row % 2 == 0) ? 0.0 : spacingX * 0.5;
        for (double x = minX + rowOffset; x <= maxX && index < requestedCount; x += spacingX) {
            const double horizontalBias = (x - minX) / std::max(maxX - minX, 1.0);
            scene.balls.push_back(makeBall(
                {x + jitter(rng), y + jitter(rng)},
                {startVx(rng) * (1.15 - 0.35 * horizontalBias), startVy(rng)},
                radius,
                makeRainbowColor(static_cast<double>(index) * 0.009 +
                                 static_cast<double>(options.seed) * 0.0001)));
            ++index;
        }
    }

    while (index < requestedCount) {
        const double x = minX + (maxX - minX) * randomX(rng);
        const double y = minY + (maxY - minY) * randomY(rng);
        const double horizontalBias = (x - minX) / std::max(maxX - minX, 1.0);
        scene.balls.push_back(makeBall(
            {x + jitter(rng), y + jitter(rng)},
            {startVx(rng) * (1.15 - 0.35 * horizontalBias), startVy(rng)},
            radius,
            makeRainbowColor(static_cast<double>(index) * 0.009 +
                             static_cast<double>(options.seed) * 0.0001)));
        ++index;
    }
}

Scene buildContainerScene(const ScenarioOptions& options) {
    const double margin =
        std::max(24.0, std::min(options.width, options.height) * 0.08);
    const double side =
        std::max(240.0, std::min(options.width, options.height) - margin * 2.0);
    Scene scene = makeBoxScene(side, side, "container");
    const Vec2 offset{
        (options.width - side) * 0.5,
        (options.height - side) * 0.5};
    scene.bounds = {offset.x, offset.y, offset.x + side, offset.y + side};
    for (Wall& wall : scene.walls) {
        wall.a += offset;
        wall.b += offset;
    }
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
    Scene scene = makeBoxScene(side, side, "packed");
    const Vec2 offset{
        (options.width - side) * 0.5,
        (options.height - side) * 0.5};
    scene.bounds = {offset.x, offset.y, offset.x + side, offset.y + side};
    for (Wall& wall : scene.walls) {
        wall.a += offset;
        wall.b += offset;
    }

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
