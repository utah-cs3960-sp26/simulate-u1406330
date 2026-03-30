#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sim/Math.h"

namespace sim {

struct Ball {
    Vec2 position;
    Vec2 velocity;
    double radius = 6.0;
    double inverseMass = 1.0;
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
    double gravity = 1400.0;
    double restitution = 0.25;
    double linearDamping = 0.05;
    double fixedDt = 1.0 / 60.0;
    double sleepBounceSpeed = 32.0;
    double sleepLinearSpeed = 32.0;
    double allowedTravelPerSubstep = 0.2;
    double overlapSlop = 0.001;
    double maxLinearSpeed = 180.0;
    int solverIterations = 10;
    int maxSubsteps = 12;
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
    struct Contact {
        enum class Type { BallBall, BallWall };

        Type type = Type::BallBall;
        std::size_t first = 0;
        std::size_t second = 0;
        Vec2 normal;
        double penetration = 0.0;
        Vec2 point;
    };

    void singleStep(double dt, StepStats& stats);
    int chooseSubsteps(double dt) const;
    std::vector<Contact> gatherContacts(double dt) const;
    void advanceBalls(double dt, StepStats& stats);
    Vec2 clampMoveAgainstWalls(const Ball& ball, const Vec2& start, const Vec2& requestedEnd) const;
    void solvePositions(const std::vector<Contact>& contacts, StepStats& stats);
    void solveVelocities(const std::vector<Contact>& contacts,
                          const std::vector<Vec2>& referenceVelocities);
    void stabilizeRestingContacts(const std::vector<Contact>& contacts);
    void enforceBounds();
    void applySleep(const std::vector<Contact>& contacts);
    std::vector<bool> computeSupportedBalls(const std::vector<Contact>& contacts) const;
    bool isSupportingNormal(const Vec2& normal) const;
    void updateContactMetrics(const std::vector<Contact>& contacts, StepStats& stats) const;
    void updateVelocityMetrics(StepStats& stats) const;
    void updateEscapeCount(StepStats& stats) const;
    double minBallRadius() const;

    Scene scene_;
    SimulationConfig config_;
    std::int64_t frameIndex_ = 0;
    StepStats lastStats_;
    std::vector<int> sleepFrames_;
    std::vector<bool> sleeping_;
};

}  // namespace sim
