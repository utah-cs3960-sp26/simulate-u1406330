#include "sim/Simulation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sim {

namespace {

constexpr double kEpsilon = 1e-9;
constexpr int kSweepIterations = 18;
constexpr int kMaxWallBouncesPerSubstep = 4;

struct SweepHit {
    bool hit = false;
    double toi = 1.0;
    std::size_t wallIndex = 0;
    Vec2 point;
    Vec2 normal;
};

Vec2 perpendicular(const Vec2& v) {
    return {-v.y, v.x};
}

Vec2 collisionNormalFromMotion(const Vec2& edge, const Vec2& motion) {
    const Vec2 wallNormal = normalize(perpendicular(edge));
    if (dot(motion, wallNormal) > 0.0) {
        return {-wallNormal.x, -wallNormal.y};
    }
    return wallNormal;
}

double pointSegmentDistanceSquared(const Vec2& point, const Wall& wall) {
    return lengthSquared(point - closestPointOnSegment(wall.a, wall.b, point));
}

SweepHit findEarliestWallHit(const Scene& scene,
                             const Ball& ball,
                             const Vec2& start,
                             const Vec2& end) {
    SweepHit bestHit;
    const Vec2 motion = end - start;
    const double minX = std::min(start.x, end.x) - ball.radius;
    const double maxX = std::max(start.x, end.x) + ball.radius;
    const double minY = std::min(start.y, end.y) - ball.radius;
    const double maxY = std::max(start.y, end.y) + ball.radius;

    for (std::size_t wallIndex = 0; wallIndex < scene.walls.size(); ++wallIndex) {
        const Wall& wall = scene.walls[wallIndex];
        if (std::max(wall.a.x, wall.b.x) < minX || std::min(wall.a.x, wall.b.x) > maxX ||
            std::max(wall.a.y, wall.b.y) < minY || std::min(wall.a.y, wall.b.y) > maxY) {
            continue;
        }
        if (pointSegmentDistanceSquared(start, wall) <= ball.radius * ball.radius + 1e-10) {
            continue;
        }

        if (segmentSegmentDistanceSquared(start, end, wall.a, wall.b) >
            ball.radius * ball.radius) {
            continue;
        }

        double low = 0.0;
        double high = 1.0;
        for (int iteration = 0; iteration < kSweepIterations; ++iteration) {
            const double mid = (low + high) * 0.5;
            const Vec2 midPoint = lerp(start, end, mid);
            if (segmentSegmentDistanceSquared(start, midPoint, wall.a, wall.b) <=
                ball.radius * ball.radius) {
                high = mid;
            } else {
                low = mid;
            }
        }

        if (high >= bestHit.toi) {
            continue;
        }

        const Vec2 impactPosition = lerp(start, end, high);
        const Vec2 closest = closestPointOnSegment(wall.a, wall.b, impactPosition);
        bestHit.hit = true;
        bestHit.toi = high;
        bestHit.wallIndex = wallIndex;
        bestHit.point = closest;
        bestHit.normal = normalize(
            impactPosition - closest,
            collisionNormalFromMotion(wall.b - wall.a, motion));
    }

    return bestHit;
}

double computeRestitution(double configuredRestitution,
                          double normalVelocity,
                          double sleepBounceSpeed) {
    if (std::abs(normalVelocity) < sleepBounceSpeed) {
        return 0.0;
    }
    return configuredRestitution;
}

void clampVelocity(Vec2& velocity, double maxSpeed) {
    if (maxSpeed <= 0.0) {
        return;
    }
    const double speedSq = lengthSquared(velocity);
    if (speedSq <= maxSpeed * maxSpeed) {
        return;
    }
    velocity *= maxSpeed / std::sqrt(speedSq);
}

}  // namespace

Simulation::Simulation(Scene scene, SimulationConfig config)
    : scene_(std::move(scene)), config_(config) {}

void Simulation::step() {
    StepStats stats;
    stats.frameIndex = frameIndex_;
    stats.substeps = chooseSubsteps(config_.fixedDt);
    const double subDt = config_.fixedDt / static_cast<double>(stats.substeps);

    for (int substep = 0; substep < stats.substeps; ++substep) {
        singleStep(subDt, stats);
    }

    std::vector<Contact> contacts = gatherContacts(0.0);
    for (int iteration = 0; iteration < 5; ++iteration) {
        bool hasOverlap = false;
        for (const Contact& contact : contacts) {
            if (contact.penetration > config_.overlapSlop) {
                hasOverlap = true;
                break;
            }
        }
        if (!hasOverlap) {
            break;
        }
        solvePositions(contacts, stats);
        enforceBounds();
        contacts = gatherContacts(0.0);
    }

    applySleep(contacts);

    updateContactMetrics(contacts, stats);
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
    const double damping = std::max(0.0, 1.0 - config_.linearDamping * dt);
    std::vector<Vec2> previousPositions;
    previousPositions.reserve(scene_.balls.size());
    for (Ball& ball : scene_.balls) {
        previousPositions.push_back(ball.position);
        ball.velocity.y += config_.gravity * dt;
        ball.velocity *= damping;
        clampVelocity(ball.velocity, config_.maxLinearSpeed);
    }

    advanceBalls(dt, stats);
    std::vector<Vec2> referenceVelocities;
    referenceVelocities.reserve(scene_.balls.size());
    for (const Ball& ball : scene_.balls) {
        referenceVelocities.push_back(ball.velocity);
    }

    for (int iteration = 0; iteration < std::max(1, config_.solverIterations); ++iteration) {
        const std::vector<Contact> contacts = gatherContacts(dt);
        if (contacts.empty()) {
            break;
        }
        solvePositions(contacts, stats);
        enforceBounds();
    }

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        scene_.balls[i].velocity = (scene_.balls[i].position - previousPositions[i]) / dt;
        clampVelocity(scene_.balls[i].velocity, config_.maxLinearSpeed);
    }

    const std::vector<Contact> contacts = gatherContacts(0.0);
    solveVelocities(contacts, referenceVelocities);
    enforceBounds();
    applySleep(contacts);
}

int Simulation::chooseSubsteps(double dt) const {
    const double radius = minBallRadius();
    if (radius <= kEpsilon) {
        return 1;
    }

    double maxSpeed = 0.0;
    for (const Ball& ball : scene_.balls) {
        maxSpeed = std::max(maxSpeed, length(ball.velocity));
    }

    const double maxTravel = std::max(radius * config_.allowedTravelPerSubstep, radius * 0.05);
    const double estimatedTravel = maxSpeed * dt + 0.5 * config_.gravity * dt * dt;
    const int desired =
        static_cast<int>(std::ceil(estimatedTravel / std::max(maxTravel, kEpsilon)));
    return std::clamp(std::max(1, desired), 1, std::max(1, config_.maxSubsteps));
}

std::vector<Simulation::Contact> Simulation::gatherContacts(double dt) const {
    std::vector<Contact> contacts;
    contacts.reserve(scene_.balls.size() * 4);

    const double cellSize = std::max(2.5 * minBallRadius(), 1.0);
    const double contactSkin = std::max(config_.overlapSlop * 4.0, minBallRadius() * 0.15);
    const int columns = std::max(
        1,
        static_cast<int>(std::ceil((scene_.bounds.maxX - scene_.bounds.minX) / cellSize)));
    const int rows = std::max(
        1,
        static_cast<int>(std::ceil((scene_.bounds.maxY - scene_.bounds.minY) / cellSize)));
    std::vector<int> cellHeads(static_cast<std::size_t>(columns * rows), -1);
    std::vector<int> next(scene_.balls.size(), -1);

    const auto cellCoord = [&](double value, double minBound, int count) {
        return std::clamp(
            static_cast<int>(std::floor((value - minBound) / cellSize)),
            0,
            std::max(0, count - 1));
    };
    const auto cellIndex = [&](int x, int y) {
        return y * columns + x;
    };

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        const Ball& ball = scene_.balls[i];
        const int cellX = cellCoord(ball.position.x, scene_.bounds.minX, columns);
        const int cellY = cellCoord(ball.position.y, scene_.bounds.minY, rows);
        const int index = cellIndex(cellX, cellY);
        next[i] = cellHeads[index];
        cellHeads[index] = static_cast<int>(i);
    }

    static constexpr int neighborOffsets[5][2] = {
        {0, 0},
        {1, 0},
        {0, 1},
        {1, 1},
        {-1, 1},
    };

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        const Ball& a = scene_.balls[i];
        const int cellX = cellCoord(a.position.x, scene_.bounds.minX, columns);
        const int cellY = cellCoord(a.position.y, scene_.bounds.minY, rows);

        for (const auto& neighborOffset : neighborOffsets) {
            const int neighborX = cellX + neighborOffset[0];
            const int neighborY = cellY + neighborOffset[1];
            if (neighborX < 0 || neighborX >= columns || neighborY < 0 || neighborY >= rows) {
                continue;
            }

            for (int current = cellHeads[cellIndex(neighborX, neighborY)]; current != -1;
                 current = next[static_cast<std::size_t>(current)]) {
                const std::size_t j = static_cast<std::size_t>(current);
                if (j <= i) {
                    continue;
                }

                const Ball& b = scene_.balls[j];
                const Vec2 delta = b.position - a.position;
                const Vec2 relativeVelocity = b.velocity - a.velocity;
                const double radii = a.radius + b.radius;
                const double contactDistance = radii + contactSkin;
                const double contactDistanceSq = contactDistance * contactDistance;
                const double distSq = lengthSquared(delta);
                const double approaching = dot(delta, relativeVelocity);
                bool speculativeContact = false;
                double distance = 0.0;
                Vec2 normal;

                if (distSq < radii * radii) {
                    distance = std::sqrt(std::max(distSq, 0.0));
                    normal = distance > kEpsilon ? delta / distance : Vec2{1.0, 0.0};
                } else if (dt > 0.0 && approaching < 0.0) {
                    const Vec2 predictedDelta = delta + relativeVelocity * dt;
                    const double predictedDistSq = lengthSquared(predictedDelta);
                    if (predictedDistSq < contactDistanceSq) {
                        speculativeContact = true;
                        distance = std::sqrt(std::max(predictedDistSq, 0.0));
                        normal = distance > kEpsilon ? predictedDelta / distance : normalize(delta);
                    } else {
                        continue;
                    }
                } else if (distSq < contactDistanceSq) {
                    distance = std::sqrt(std::max(distSq, 0.0));
                    normal = distance > kEpsilon ? delta / distance : Vec2{1.0, 0.0};
                } else {
                    continue;
                }

                Contact contact;
                contact.type = Contact::Type::BallBall;
                contact.first = i;
                contact.second = j;
                contact.normal = normal;
                contact.penetration = speculativeContact ? (contactDistance - distance)
                                                         : (radii - distance);
                contact.point = a.position + contact.normal * a.radius;
                contacts.push_back(contact);
            }
        }

        for (std::size_t wallIndex = 0; wallIndex < scene_.walls.size(); ++wallIndex) {
            const Wall& wall = scene_.walls[wallIndex];
            const Vec2 closest = closestPointOnSegment(wall.a, wall.b, a.position);
            const Vec2 delta = a.position - closest;
            const double contactDistance = a.radius + contactSkin;
            const double distSq = lengthSquared(delta);
            if (distSq >= contactDistance * contactDistance) {
                continue;
            }

            const double distance = std::sqrt(std::max(distSq, 0.0));
            Contact contact;
            contact.type = Contact::Type::BallWall;
            contact.first = i;
            contact.second = wallIndex;
            contact.normal = distance > kEpsilon
                                 ? delta / distance
                                 : collisionNormalFromMotion(wall.b - wall.a, a.velocity);
            contact.penetration = a.radius - distance;
            contact.point = closest;
            contacts.push_back(contact);
        }
    }

    return contacts;
}

void Simulation::advanceBalls(double dt, StepStats& stats) {
    for (Ball& ball : scene_.balls) {
        Vec2 position = ball.position;
        Vec2 velocity = ball.velocity;
        double remainingTime = dt;

        for (int bounce = 0; bounce < kMaxWallBouncesPerSubstep && remainingTime > 1e-8; ++bounce) {
            const Vec2 target = position + velocity * remainingTime;
            const SweepHit hit = findEarliestWallHit(scene_, ball, position, target);
            if (!hit.hit) {
                position = target;
                remainingTime = 0.0;
                break;
            }

            const double advanceTime = remainingTime * hit.toi;
            position += velocity * advanceTime;
            position = hit.point + hit.normal * (ball.radius + config_.overlapSlop);

            const double normalVelocity = dot(velocity, hit.normal);
            if (normalVelocity < 0.0) {
                const double restitution = computeRestitution(
                    config_.restitution,
                    normalVelocity,
                    config_.sleepBounceSpeed);
                velocity -= (1.0 + restitution) * normalVelocity * hit.normal;
                clampVelocity(velocity, config_.maxLinearSpeed);
            }

            remainingTime *= (1.0 - hit.toi);
            remainingTime = std::max(0.0, remainingTime - dt * 1e-4);
            ++stats.ballWallContacts;
        }

        if (remainingTime > 1e-8) {
            position += velocity * remainingTime;
        }

        ball.position = position;
        ball.velocity = velocity;
    }
}

void Simulation::solvePositions(const std::vector<Contact>& contacts, StepStats&) {
    const double separationBias = std::max(config_.overlapSlop, minBallRadius() * 0.08);

    for (auto it = contacts.rbegin(); it != contacts.rend(); ++it) {
        const Contact& contact = *it;
        if (contact.type != Contact::Type::BallBall) {
            continue;
        }

        const double correction =
            std::max(0.0, contact.penetration + separationBias - config_.overlapSlop);
        if (correction <= 0.0) {
            continue;
        }

        Ball& a = scene_.balls[contact.first];
        Ball& b = scene_.balls[contact.second];
        const double inverseMassSum = a.inverseMass + b.inverseMass;
        if (inverseMassSum <= kEpsilon) {
            continue;
        }

        const double aShare = a.inverseMass / inverseMassSum;
        const double bShare = b.inverseMass / inverseMassSum;
        a.position -= contact.normal * (correction * aShare);
        b.position += contact.normal * (correction * bShare);
    }

    for (const Contact& contact : contacts) {
        if (contact.type != Contact::Type::BallWall) {
            continue;
        }

        const double correction =
            std::max(0.0, contact.penetration + separationBias - config_.overlapSlop);
        if (correction <= 0.0) {
            continue;
        }

        scene_.balls[contact.first].position += contact.normal * correction;
    }
}

void Simulation::solveVelocities(const std::vector<Contact>& contacts,
                                 const std::vector<Vec2>& referenceVelocities) {
    for (const Contact& contact : contacts) {
        if (contact.type != Contact::Type::BallWall) {
            continue;
        }

        Ball& ball = scene_.balls[contact.first];
        const double currentNormalVelocity = dot(ball.velocity, contact.normal);
        if (currentNormalVelocity >= 0.0) {
            continue;
        }

        const double referenceNormalVelocity =
            dot(referenceVelocities[contact.first], contact.normal);
        const double restitution = computeRestitution(
            config_.restitution,
            referenceNormalVelocity,
            config_.sleepBounceSpeed);
        const double desiredNormalVelocity =
            std::max(0.0, -referenceNormalVelocity * restitution);
        ball.velocity += (desiredNormalVelocity - currentNormalVelocity) * contact.normal;
        clampVelocity(ball.velocity, config_.maxLinearSpeed);
    }

    for (auto it = contacts.rbegin(); it != contacts.rend(); ++it) {
        const Contact& contact = *it;
        if (contact.type != Contact::Type::BallBall) {
            continue;
        }

        Ball& a = scene_.balls[contact.first];
        Ball& b = scene_.balls[contact.second];
        const Vec2 currentRelativeVelocity = b.velocity - a.velocity;
        const double currentNormalVelocity = dot(currentRelativeVelocity, contact.normal);
        if (currentNormalVelocity >= 0.0) {
            continue;
        }

        const double inverseMassSum = a.inverseMass + b.inverseMass;
        if (inverseMassSum <= kEpsilon) {
            continue;
        }

        const Vec2 referenceRelativeVelocity =
            referenceVelocities[contact.second] - referenceVelocities[contact.first];
        const double referenceNormalVelocity = dot(referenceRelativeVelocity, contact.normal);
        const double restitution = computeRestitution(
            config_.restitution,
            referenceNormalVelocity,
            config_.sleepBounceSpeed);
        const double desiredNormalVelocity =
            std::max(0.0, -referenceNormalVelocity * restitution);
        const double impulse =
            (desiredNormalVelocity - currentNormalVelocity) / inverseMassSum;
        a.velocity -= contact.normal * (impulse * a.inverseMass);
        b.velocity += contact.normal * (impulse * b.inverseMass);
        clampVelocity(a.velocity, config_.maxLinearSpeed);
        clampVelocity(b.velocity, config_.maxLinearSpeed);
    }
}

void Simulation::enforceBounds() {
    for (Ball& ball : scene_.balls) {
        const double minX = scene_.bounds.minX + ball.radius;
        const double maxX = scene_.bounds.maxX - ball.radius;
        const double minY = scene_.bounds.minY + ball.radius;
        const double maxY = scene_.bounds.maxY - ball.radius;

        if (ball.position.x < minX) {
            ball.position.x = minX;
            if (ball.velocity.x < 0.0) {
                const double restitution = computeRestitution(
                    config_.restitution,
                    ball.velocity.x,
                    config_.sleepBounceSpeed);
                ball.velocity.x = -ball.velocity.x * restitution;
            }
        } else if (ball.position.x > maxX) {
            ball.position.x = maxX;
            if (ball.velocity.x > 0.0) {
                const double restitution = computeRestitution(
                    config_.restitution,
                    -ball.velocity.x,
                    config_.sleepBounceSpeed);
                ball.velocity.x = -ball.velocity.x * restitution;
            }
        }

        if (ball.position.y < minY) {
            ball.position.y = minY;
            if (ball.velocity.y < 0.0) {
                const double restitution = computeRestitution(
                    config_.restitution,
                    ball.velocity.y,
                    config_.sleepBounceSpeed);
                ball.velocity.y = -ball.velocity.y * restitution;
            }
        } else if (ball.position.y > maxY) {
            ball.position.y = maxY;
            if (ball.velocity.y > 0.0) {
                const double restitution = computeRestitution(
                    config_.restitution,
                    -ball.velocity.y,
                    config_.sleepBounceSpeed);
                ball.velocity.y = -ball.velocity.y * restitution;
            }
        }
    }
}

void Simulation::applySleep(const std::vector<Contact>& contacts) {
    std::vector<bool> hasContact(scene_.balls.size(), false);
    std::vector<bool> hasDeepContact(scene_.balls.size(), false);
    const double quietSpeedSq = config_.sleepLinearSpeed * config_.sleepLinearSpeed;
    const double deepPenetration = std::max(config_.overlapSlop * 4.0, 0.01);

    for (const Contact& contact : contacts) {
        hasContact[contact.first] = true;
        if (contact.penetration > deepPenetration) {
            hasDeepContact[contact.first] = true;
        }
        if (contact.type == Contact::Type::BallBall) {
            hasContact[contact.second] = true;
            if (contact.penetration > deepPenetration) {
                hasDeepContact[contact.second] = true;
            }
        }
    }

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        if (!hasContact[i] || hasDeepContact[i]) {
            continue;
        }
        if (lengthSquared(scene_.balls[i].velocity) <= quietSpeedSq) {
            scene_.balls[i].velocity = {0.0, 0.0};
        }
    }
}

void Simulation::updateContactMetrics(const std::vector<Contact>& contacts, StepStats& stats) const {
    for (const Contact& contact : contacts) {
        if (contact.type == Contact::Type::BallBall) {
            ++stats.ballBallContacts;
        } else {
            ++stats.ballWallContacts;
        }
        if (contact.penetration > config_.overlapSlop) {
            ++stats.overlapCount;
        }
        stats.maxPenetration = std::max(stats.maxPenetration, contact.penetration);
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
