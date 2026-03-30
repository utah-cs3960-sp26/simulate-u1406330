#include "sim/Simulation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sim {

namespace {

constexpr double kEpsilon = 1e-9;

Vec2 perpendicular(const Vec2& v) {
    return {-v.y, v.x};
}

Vec2 outwardWallNormal(const Wall& wall, const Vec2& referencePoint) {
    Vec2 normal = normalize(perpendicular(wall.b - wall.a));
    if (dot(referencePoint - wall.a, normal) < 0.0) {
        normal *= -1.0;
    }
    return normal;
}

}  // namespace

Simulation::Simulation(Scene scene, SimulationConfig config)
    : scene_(std::move(scene)),
      config_(config) {
    for (Ball& ball : scene_.balls) {
        if (lengthSquared(ball.previousPosition - ball.position) <= kEpsilon) {
            ball.previousPosition = ball.position - ball.velocity * config_.fixedDt;
        }
    }
}

void Simulation::step() {
    StepStats stats;
    stats.frameIndex = frameIndex_;
    stats.substeps = chooseSubsteps(config_.fixedDt);
    const double subDt =
        stats.substeps > 0 ? config_.fixedDt / static_cast<double>(stats.substeps) : config_.fixedDt;

    for (int substep = 0; substep < stats.substeps; ++substep) {
        singleStep(subDt, stats);
    }

    for (Ball& ball : scene_.balls) {
        if (subDt <= kEpsilon) {
            ball.velocity = {0.0, 0.0};
        } else {
            ball.velocity = ball.position - ball.previousPosition;
        }
    }

    updateContactMetrics(stats);
    updateVelocityMetrics(stats);
    updateEscapeCount(stats);
    lastStats_ = stats;
    ++frameIndex_;
}

void Simulation::stepMany(int steps) {
    for (int i = 0; i < steps; ++i) {
        step();
    }
}

void Simulation::singleStep(double dt, StepStats& stats) {
    solveBallCollisions(stats);
    advanceBalls(dt);
    solveWallOverlaps(stats);
    enforceBounds();
}

int Simulation::chooseSubsteps(double dt) const {
    static_cast<void>(dt);
    return std::max(1, config_.maxSubsteps);
}

void Simulation::solveBallCollisions(StepStats& stats) {
    if (scene_.balls.size() < 2) {
        return;
    }

    const double minRadius = minBallRadius();
    const double diameter = std::max(2.0 * minRadius, 1.0);
    const double worldWidth = std::max(scene_.bounds.maxX - scene_.bounds.minX, diameter);
    const double worldHeight = std::max(scene_.bounds.maxY - scene_.bounds.minY, diameter);
    const int columns =
        std::max(1, static_cast<int>(std::ceil(worldWidth / diameter)));
    const int rows =
        std::max(1, static_cast<int>(std::ceil(worldHeight / diameter)));
    std::vector<std::vector<std::size_t>> grid(static_cast<std::size_t>(columns * rows));

    const auto cellCoord = [&](double value, double minBound, int count) {
        return std::clamp(
            static_cast<int>(std::floor((value - minBound) / diameter)),
            0,
            std::max(0, count - 1));
    };
    const auto cellIndex = [&](int x, int y) {
        return static_cast<std::size_t>(y * columns + x);
    };

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        const Ball& ball = scene_.balls[i];
        const int cellX = cellCoord(ball.position.x, scene_.bounds.minX, columns);
        const int cellY = cellCoord(ball.position.y, scene_.bounds.minY, rows);
        grid[cellIndex(cellX, cellY)].push_back(i);
    }

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < columns; ++x) {
            const std::vector<std::size_t>& currentCell = grid[cellIndex(x, y)];
            for (std::size_t atomIdx : currentCell) {
                for (int neighborY = std::max(0, y - 1); neighborY <= std::min(rows - 1, y + 1);
                     ++neighborY) {
                    for (int neighborX = std::max(0, x - 1);
                         neighborX <= std::min(columns - 1, x + 1);
                         ++neighborX) {
                        const std::vector<std::size_t>& neighborCell =
                            grid[cellIndex(neighborX, neighborY)];
                        for (std::size_t otherIdx : neighborCell) {
                            if (otherIdx <= atomIdx) {
                                continue;
                            }

                            Ball& a = scene_.balls[atomIdx];
                            Ball& b = scene_.balls[otherIdx];
                            const Vec2 delta = a.position - b.position;
                            const double minDistance = a.radius + b.radius;
                            const double distanceSq = lengthSquared(delta);
                            if (distanceSq >= minDistance * minDistance) {
                                continue;
                            }

                            const double distance = std::sqrt(std::max(distanceSq, 0.0));
                            const Vec2 normal =
                                distance > kEpsilon ? delta / distance : Vec2{1.0, 0.0};
                            const double penetration = minDistance - distance;
                            const double inverseMassSum = a.inverseMass + b.inverseMass;
                            if (inverseMassSum <= kEpsilon) {
                                continue;
                            }

                            const double aShare = a.inverseMass / inverseMassSum;
                            const double bShare = b.inverseMass / inverseMassSum;
                            const Vec2 correction = normal * penetration;
                            a.position += correction * aShare;
                            b.position -= correction * bShare;
                            ++stats.ballBallContacts;
                        }
                    }
                }
            }
        }
    }
}

void Simulation::advanceBalls(double dt) {
    for (Ball& ball : scene_.balls) {
        const Vec2 displacement = ball.position - ball.previousPosition;
        const Vec2 acceleration{0.0, config_.gravity};
        const Vec2 nextPosition =
            ball.position + displacement +
            (acceleration - displacement * config_.linearDamping) * (dt * dt);
        ball.previousPosition = ball.position;
        ball.position = nextPosition;
    }
}

void Simulation::solveWallOverlaps(StepStats& stats) {
    for (Ball& ball : scene_.balls) {
        for (const Wall& wall : scene_.walls) {
            const Vec2 wallVector = wall.b - wall.a;
            const double wallLength = length(wallVector);
            if (wallLength <= kEpsilon) {
                continue;
            }

            const Vec2 tangent = wallVector / wallLength;
            Vec2 wallNormal = outwardWallNormal(wall, ball.previousPosition);
            const Vec2 motion = ball.position - ball.previousPosition;
            const double startDistance = dot(ball.previousPosition - wall.a, wallNormal);
            const double endDistance = dot(ball.position - wall.a, wallNormal);
            if (startDistance >= ball.radius && endDistance < ball.radius && endDistance < startDistance) {
                const double denominator = startDistance - endDistance;
                if (denominator > kEpsilon) {
                    const double t =
                        std::clamp((startDistance - ball.radius) / denominator, 0.0, 1.0);
                    const Vec2 hitCenter = ball.previousPosition + motion * t;
                    const double wallCoordinate = dot(hitCenter - wall.a, tangent);
                    if (wallCoordinate >= -ball.radius && wallCoordinate <= wallLength + ball.radius) {
                        ball.position += wallNormal * (ball.radius - endDistance);
                        ++stats.ballWallContacts;
                        continue;
                    }
                }
            }

            const Vec2 closest = closestPointOnSegment(wall.a, wall.b, ball.position);
            const Vec2 delta = ball.position - closest;
            const double distanceSq = lengthSquared(delta);
            if (distanceSq >= ball.radius * ball.radius) {
                continue;
            }

            const double distance = std::sqrt(std::max(distanceSq, 0.0));
            const Vec2 normal = outwardWallNormal(wall, ball.previousPosition);
            const double penetration = ball.radius - distance;
            ball.position += normal * penetration;
            ++stats.ballWallContacts;
        }
    }
}

void Simulation::enforceBounds() {
    for (Ball& ball : scene_.balls) {
        const double minX = scene_.bounds.minX + ball.radius;
        const double maxX = scene_.bounds.maxX - ball.radius;
        const double minY = scene_.bounds.minY + ball.radius;
        const double maxY = scene_.bounds.maxY - ball.radius;

        ball.position.x = std::clamp(ball.position.x, minX, maxX);
        ball.position.y = std::clamp(ball.position.y, minY, maxY);
    }
}

void Simulation::updateContactMetrics(StepStats& stats) const {
    stats.maxPenetration = 0.0;
    stats.overlapCount = 0;

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        const Ball& a = scene_.balls[i];
        for (std::size_t j = i + 1; j < scene_.balls.size(); ++j) {
            const Ball& b = scene_.balls[j];
            const double penetration =
                a.radius + b.radius - length(b.position - a.position);
            if (penetration > 0.0) {
                ++stats.overlapCount;
                stats.maxPenetration = std::max(stats.maxPenetration, penetration);
            }
        }

        for (const Wall& wall : scene_.walls) {
            const Vec2 closest = closestPointOnSegment(wall.a, wall.b, a.position);
            const double penetration = a.radius - length(a.position - closest);
            if (penetration > 0.0) {
                ++stats.overlapCount;
                stats.maxPenetration = std::max(stats.maxPenetration, penetration);
            }
        }
    }
}

void Simulation::updateVelocityMetrics(StepStats& stats) const {
    stats.kineticEnergy = 0.0;
    stats.maxSpeed = 0.0;
    for (const Ball& ball : scene_.balls) {
        const double mass = ball.inverseMass > kEpsilon ? 1.0 / ball.inverseMass : 0.0;
        const double speedSq = lengthSquared(ball.velocity);
        stats.kineticEnergy += 0.5 * mass * speedSq;
        stats.maxSpeed = std::max(stats.maxSpeed, std::sqrt(speedSq));
    }
}

void Simulation::updateEscapeCount(StepStats& stats) const {
    stats.escapedBalls = 0;
    for (const Ball& ball : scene_.balls) {
        if (ball.position.x - ball.radius < scene_.bounds.minX - 1e-6 ||
            ball.position.x + ball.radius > scene_.bounds.maxX + 1e-6 ||
            ball.position.y - ball.radius < scene_.bounds.minY - 1e-6 ||
            ball.position.y + ball.radius > scene_.bounds.maxY + 1e-6) {
            ++stats.escapedBalls;
        }
    }
}

double Simulation::minBallRadius() const {
    if (scene_.balls.empty()) {
        return 1.0;
    }

    double radius = std::numeric_limits<double>::infinity();
    for (const Ball& ball : scene_.balls) {
        radius = std::min(radius, ball.radius);
    }
    return std::max(radius, 1e-3);
}

}  // namespace sim
