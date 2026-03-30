#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sim/Math.h"

namespace sim {

struct ColorRGBA {
    std::uint8_t r = 214;
    std::uint8_t g = 92;
    std::uint8_t b = 45;
    std::uint8_t a = 255;
};

struct Ball {
    Vec2 position;
    Vec2 previousPosition;
    Vec2 velocity;
    double radius = 6.0;
    double inverseMass = 1.0;
    ColorRGBA color{};
};

struct Wall {
    Vec2 a;
    Vec2 b;
};

struct WorldBounds {
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
};

struct Scene {
    std::string name;
    WorldBounds bounds;
    std::vector<Ball> balls;
    std::vector<Wall> walls;
};

struct SimulationConfig {
    double gravity = 120.0;
    double restitution = 0.25;
    double linearDamping = 40.0;
    double fixedDt = 1.0 / 60.0;
    double sleepBounceSpeed = 12.0;
    double sleepLinearSpeed = 8.0;
    double allowedTravelPerSubstep = 0.2;
    double overlapSlop = 0.001;
    double maxLinearSpeed = 180.0;
    int solverIterations = 8;
    int maxSubsteps = 8;
};

struct StepStats {
    std::int64_t frameIndex = 0;
    int substeps = 0;
    int ballBallContacts = 0;
    int ballWallContacts = 0;
    int overlapCount = 0;
    int escapedBalls = 0;
    double maxPenetration = 0.0;
    double kineticEnergy = 0.0;
    double maxSpeed = 0.0;
};

class Simulation {
public:
    Simulation(Scene scene, SimulationConfig config);

    void step();
    void stepMany(int steps);

    const Scene& scene() const { return scene_; }
    Scene& mutableScene() { return scene_; }
    const SimulationConfig& config() const { return config_; }
    StepStats lastStats() const { return lastStats_; }

private:
    void singleStep(double dt, StepStats& stats);
    int chooseSubsteps(double dt) const;
    void solveBallCollisions(StepStats& stats);
    void advanceBalls(double dt);
    void solveWallOverlaps(StepStats& stats);
    void enforceBounds();
    void updateContactMetrics(StepStats& stats) const;
    void updateVelocityMetrics(StepStats& stats) const;
    void updateEscapeCount(StepStats& stats) const;
    double minBallRadius() const;

    Scene scene_;
    SimulationConfig config_;
    std::int64_t frameIndex_ = 0;
    StepStats lastStats_;
};

}  // namespace sim
