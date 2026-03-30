#include "sim/Scenario.h"

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

void addBoundaryWalls(Scene& scene, double width, double height) {
    scene.walls.push_back({{0.0, 0.0}, {width, 0.0}});
    scene.walls.push_back({{width, 0.0}, {width, height}});
    scene.walls.push_back({{width, height}, {0.0, height}});
    scene.walls.push_back({{0.0, height}, {0.0, 0.0}});
}

void addContainerBalls(Scene& scene, const ScenarioOptions& options) {
    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<double> jitter(-0.04 * options.radius, 0.04 * options.radius);
    std::uniform_real_distribution<double> startVx(-18.0, 18.0);
    std::uniform_real_distribution<double> startVy(-6.0, 6.0);

    const double spacing = options.radius * 2.28;
    const double minX = 260.0;
    const double maxX = options.width - 260.0;
    const double startY = 36.0;
    const double maxY = 316.0;
    const int columns =
        std::max(1, static_cast<int>(std::floor((maxX - minX) / std::max(1.0, spacing))));

    for (int index = 0; index < options.ballCount; ++index) {
        const int column = index % columns;
        const int row = index / columns;
        const double x = minX + options.radius + column * spacing + jitter(rng);
        const double y = startY + options.radius + row * spacing + jitter(rng);
        if (y + options.radius >= maxY) {
            break;
        }
        scene.balls.push_back(makeBall(
            {x, y},
            {startVx(rng), startVy(rng)},
            options.radius,
            makeRainbowColor(static_cast<double>(index) * 0.017 + static_cast<double>(options.seed) * 0.0001)));
    }
}

Scene buildContainerScene(const ScenarioOptions& options) {
    Scene scene;
    scene.name = "container";
    scene.bounds = {0.0, 0.0, options.width, options.height};
    addBoundaryWalls(scene, options.width, options.height);

    const double basinY = options.height - 92.0;
    const double leftInnerX = 420.0;
    const double rightInnerX = options.width - leftInnerX;

    scene.walls.push_back({{220.0, 220.0}, {leftInnerX, basinY}});
    scene.walls.push_back({{options.width - 220.0, 220.0}, {rightInnerX, basinY}});
    scene.walls.push_back({{leftInnerX, basinY}, {rightInnerX, basinY}});
    scene.walls.push_back({{260.0, 420.0}, {520.0, 420.0}});
    scene.walls.push_back({{options.width - 520.0, 420.0}, {options.width - 260.0, 420.0}});

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
    if (options.name == "gap") {
        return buildGapScene(options);
    }
    throw std::invalid_argument("unknown scenario: " + options.name);
}

}  // namespace sim
