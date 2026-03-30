#include "sim/Simulation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sim {

namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kContactSkinScale = 0.1;
constexpr int kSleepFramesThreshold = 30;

Vec2 perpendicular(const Vec2& v) {
    return {-v.y, v.x};
}

double speedForBounceThreshold(double normalComponent, double dt) {
    if (dt <= kEpsilon) {
        return 0.0;
    }
    return std::abs(normalComponent) / dt;
}

double computeRestitution(double configuredRestitution,
                          double normalComponent,
                          double dt,
                          double sleepBounceSpeed) {
    if (speedForBounceThreshold(normalComponent, dt) < sleepBounceSpeed) {
        return 0.0;
    }
    return configuredRestitution;
}

void clampDisplacement(Vec2& displacement, double maxTravel) {
    if (maxTravel <= 0.0) {
        return;
    }

    const double displacementSq = lengthSquared(displacement);
    if (displacementSq <= maxTravel * maxTravel) {
        return;
    }

    displacement *= maxTravel / std::sqrt(displacementSq);
}

void writeVelocityFromDisplacement(Ball& ball, double dt) {
    if (dt <= kEpsilon) {
        ball.velocity = {0.0, 0.0};
        return;
    }
    ball.velocity = (ball.position - ball.previousPosition) / dt;
}

void writeDisplacementFromVelocity(Ball& ball, const Vec2& velocity, double dt) {
    ball.previousPosition = ball.position - velocity * dt;
    ball.velocity = velocity;
}

Vec2 wallNormalTowardBall(const Wall& wall, const Vec2& referencePoint, const Vec2& fallbackMotion) {
    Vec2 normal = normalize(perpendicular(wall.b - wall.a));
    if (dot(referencePoint - wall.a, normal) < 0.0) {
        normal *= -1.0;
    }
    if (lengthSquared(fallbackMotion) > kEpsilon && dot(fallbackMotion, normal) > 0.0) {
        normal *= -1.0;
    }
    return normal;
}

}  // namespace

Simulation::Simulation(Scene scene, SimulationConfig config)
    : scene_(std::move(scene)),
      config_(config),
      sleepFrames_(scene_.balls.size(), 0),
      sleeping_(scene_.balls.size(), false) {
    for (Ball& ball : scene_.balls) {
        if (lengthSquared(ball.previousPosition - ball.position) <= kEpsilon) {
            ball.previousPosition = ball.position - ball.velocity * config_.fixedDt;
        }
    }
}

void Simulation::step() {
    if (sleepFrames_.size() != scene_.balls.size()) {
        sleepFrames_.assign(scene_.balls.size(), 0);
        sleeping_.assign(scene_.balls.size(), false);
    }

    StepStats stats;
    stats.frameIndex = frameIndex_;
    stats.substeps = chooseSubsteps(config_.fixedDt);
    const double subDt = config_.fixedDt / static_cast<double>(stats.substeps);

    for (int substep = 0; substep < stats.substeps; ++substep) {
        singleStep(subDt, stats);
    }

    const std::vector<Contact> contacts = gatherContacts(0.0);
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
    advanceBalls(dt, stats);

    for (int iteration = 0; iteration < std::max(1, config_.solverIterations); ++iteration) {
        const std::vector<Contact> contacts = gatherContacts(dt);
        if (contacts.empty()) {
            break;
        }
        solvePositions(contacts, stats);
        enforceBounds();
    }

    const std::vector<Contact> contacts = gatherContacts(dt);
    solveVelocities(contacts, {});
    stabilizeRestingContacts(contacts);

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        Ball& ball = scene_.balls[i];
        if (sleeping_[i]) {
            ball.previousPosition = ball.position;
            ball.velocity = {0.0, 0.0};
            continue;
        }
        writeVelocityFromDisplacement(ball, dt);
    }

    applySleep(contacts);
    enforceBounds();
    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        if (sleeping_[i]) {
            scene_.balls[i].previousPosition = scene_.balls[i].position;
            scene_.balls[i].velocity = {0.0, 0.0};
            continue;
        }
        writeVelocityFromDisplacement(scene_.balls[i], dt);
    }
}

int Simulation::chooseSubsteps(double dt) const {
    const double radius = minBallRadius();
    if (radius <= kEpsilon) {
        return 1;
    }

    double maxSpeed = 0.0;
    for (const Ball& ball : scene_.balls) {
        const Vec2 displacement = ball.position - ball.previousPosition;
        const double speed = dt > kEpsilon ? length(displacement) / dt : length(ball.velocity);
        maxSpeed = std::max(maxSpeed, speed);
    }

    const double maxTravel = std::max(radius * config_.allowedTravelPerSubstep, radius * 0.1);
    const int desired =
        static_cast<int>(std::ceil((maxSpeed * dt) / std::max(maxTravel, kEpsilon)));
    return std::clamp(std::max(1, desired), 1, std::max(1, config_.maxSubsteps));
}

std::vector<Simulation::Contact> Simulation::gatherContacts(double) const {
    std::vector<Contact> contacts;
    contacts.reserve(scene_.balls.size() * 6);

    const double minRadius = minBallRadius();
    const double cellSize = std::max(2.5 * minRadius, 1.0);
    const double contactSkin = std::max(config_.overlapSlop * 2.0, minRadius * kContactSkinScale);
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

        for (const auto& offset : neighborOffsets) {
            const int neighborX = cellX + offset[0];
            const int neighborY = cellY + offset[1];
            if (neighborX < 0 || neighborX >= columns || neighborY < 0 || neighborY >= rows) {
                continue;
            }

            for (int current = cellHeads[cellIndex(neighborX, neighborY)];
                 current != -1;
                 current = next[static_cast<std::size_t>(current)]) {
                const std::size_t j = static_cast<std::size_t>(current);
                if (j <= i) {
                    continue;
                }

                const Ball& b = scene_.balls[j];
                const Vec2 delta = b.position - a.position;
                const double radii = a.radius + b.radius;
                const double distanceSq = lengthSquared(delta);
                const double contactDistance = radii + contactSkin;
                if (distanceSq > contactDistance * contactDistance) {
                    continue;
                }

                const double distance = std::sqrt(std::max(0.0, distanceSq));
                Contact contact;
                contact.type = Contact::Type::BallBall;
                contact.first = i;
                contact.second = j;
                contact.normal = distance > kEpsilon ? delta / distance : Vec2{1.0, 0.0};
                contact.penetration = radii - distance;
                contact.point = a.position + contact.normal * a.radius;
                contacts.push_back(contact);
            }
        }

        for (std::size_t wallIndex = 0; wallIndex < scene_.walls.size(); ++wallIndex) {
            const Wall& wall = scene_.walls[wallIndex];
            const Vec2 closest = closestPointOnSegment(wall.a, wall.b, a.position);
            const Vec2 delta = a.position - closest;
            const double distanceSq = lengthSquared(delta);
            const double contactDistance = a.radius + contactSkin;
            if (distanceSq > contactDistance * contactDistance) {
                continue;
            }

            const double distance = std::sqrt(std::max(0.0, distanceSq));
            Contact contact;
            contact.type = Contact::Type::BallWall;
            contact.first = i;
            contact.second = wallIndex;
            contact.normal =
                distance > kEpsilon
                    ? delta / distance
                    : wallNormalTowardBall(wall, a.position, a.position - a.previousPosition);
            contact.penetration = a.radius - distance;
            contact.point = closest;
            contacts.push_back(contact);
        }
    }

    return contacts;
}

void Simulation::advanceBalls(double dt, StepStats&) {
    const double damping = std::max(0.0, 1.0 - config_.linearDamping * dt);
    const double maxTravel = std::max(0.0, config_.maxLinearSpeed * dt);

    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        Ball& ball = scene_.balls[i];
        if (sleeping_[i]) {
            ball.previousPosition = ball.position;
            ball.velocity = {0.0, 0.0};
            continue;
        }

        Vec2 displacement = (ball.position - ball.previousPosition) * damping;
        clampDisplacement(displacement, maxTravel);
        const Vec2 current = ball.position;
        ball.position = current + displacement + Vec2{0.0, config_.gravity * dt * dt};
        ball.previousPosition = current;
    }
}

Vec2 Simulation::clampMoveAgainstWalls(const Ball&, const Vec2&, const Vec2& requestedEnd) const {
    return requestedEnd;
}

void Simulation::solvePositions(const std::vector<Contact>& contacts, StepStats&) {
    for (const Contact& contact : contacts) {
        if (contact.type == Contact::Type::BallBall) {
            if (contact.penetration <= 0.0) {
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
            const Vec2 correction = contact.normal * (contact.penetration + config_.overlapSlop);
            a.position -= correction * aShare;
            b.position += correction * bShare;

            const Vec2 velocityA = a.position - a.previousPosition;
            const Vec2 velocityB = b.position - b.previousPosition;
            const Vec2 relativeVelocity = velocityB - velocityA;
            const double normalVelocity = dot(relativeVelocity, contact.normal);
            if (normalVelocity < 0.0) {
                const double restitution = computeRestitution(
                    config_.restitution,
                    normalVelocity,
                    config_.fixedDt,
                    config_.sleepBounceSpeed);
                const double impulse = (-(1.0 + restitution) * normalVelocity) / inverseMassSum;
                writeDisplacementFromVelocity(
                    a,
                    velocityA - contact.normal * (impulse * a.inverseMass),
                    1.0);
                writeDisplacementFromVelocity(
                    b,
                    velocityB + contact.normal * (impulse * b.inverseMass),
                    1.0);
            }
            continue;
        }

        if (contact.penetration <= 0.0) {
            continue;
        }

        Ball& ball = scene_.balls[contact.first];
        ball.position += contact.normal * (contact.penetration + config_.overlapSlop);
        const Vec2 displacement = ball.position - ball.previousPosition;
        const double normalVelocity = dot(displacement, contact.normal);
        if (normalVelocity < 0.0) {
            const double restitution = computeRestitution(
                config_.restitution,
                normalVelocity,
                config_.fixedDt,
                config_.sleepBounceSpeed);
            const Vec2 reflected =
                displacement - (1.0 + restitution) * normalVelocity * contact.normal;
            writeDisplacementFromVelocity(ball, reflected, 1.0);
        }
    }
}

void Simulation::solveVelocities(const std::vector<Contact>&,
                                 const std::vector<Vec2>&) {
}

void Simulation::stabilizeRestingContacts(const std::vector<Contact>& contacts) {
    const double quietSpeed = std::max(config_.sleepLinearSpeed, config_.sleepBounceSpeed * 0.5);
    const double quietSpeedSq = quietSpeed * quietSpeed;

    for (const Contact& contact : contacts) {
        if (contact.type != Contact::Type::BallWall || !isSupportingNormal(contact.normal)) {
            continue;
        }

        Ball& ball = scene_.balls[contact.first];
        const Vec2 displacement = ball.position - ball.previousPosition;
        if (lengthSquared(displacement) > quietSpeedSq * config_.fixedDt * config_.fixedDt) {
            continue;
        }

        const double normalComponent = dot(displacement, contact.normal);
        if (normalComponent > 0.0) {
            continue;
        }

        const Vec2 stabilized = displacement - contact.normal * normalComponent;
        ball.previousPosition = ball.position - stabilized;
    }
}

void Simulation::enforceBounds() {
    for (Ball& ball : scene_.balls) {
        Vec2 displacement = ball.position - ball.previousPosition;
        const double minX = scene_.bounds.minX + ball.radius;
        const double maxX = scene_.bounds.maxX - ball.radius;
        const double minY = scene_.bounds.minY + ball.radius;
        const double maxY = scene_.bounds.maxY - ball.radius;

        if (ball.position.x < minX) {
            ball.position.x = minX;
            if (displacement.x < 0.0) {
                displacement.x = -displacement.x * config_.restitution;
            }
        } else if (ball.position.x > maxX) {
            ball.position.x = maxX;
            if (displacement.x > 0.0) {
                displacement.x = -displacement.x * config_.restitution;
            }
        }

        if (ball.position.y < minY) {
            ball.position.y = minY;
            if (displacement.y < 0.0) {
                displacement.y = -displacement.y * config_.restitution;
            }
        } else if (ball.position.y > maxY) {
            ball.position.y = maxY;
            if (displacement.y > 0.0) {
                displacement.y = -displacement.y * config_.restitution;
            }
        }

        ball.previousPosition = ball.position - displacement;
    }
}

void Simulation::applySleep(const std::vector<Contact>& contacts) {
    std::vector<bool> hasContact(scene_.balls.size(), false);
    const std::vector<bool> supported = computeSupportedBalls(contacts);

    for (const Contact& contact : contacts) {
        hasContact[contact.first] = true;
        if (contact.type == Contact::Type::BallBall) {
            hasContact[contact.second] = true;
        }
    }

    const double quietSpeedSq = config_.sleepLinearSpeed * config_.sleepLinearSpeed;
    for (std::size_t i = 0; i < scene_.balls.size(); ++i) {
        const double speedSq = lengthSquared(scene_.balls[i].velocity);
        if (!hasContact[i] || !supported[i]) {
            sleeping_[i] = false;
            sleepFrames_[i] = 0;
            continue;
        }

        if (speedSq <= quietSpeedSq) {
            ++sleepFrames_[i];
        } else {
            sleeping_[i] = false;
            sleepFrames_[i] = 0;
        }

        if (sleepFrames_[i] >= kSleepFramesThreshold) {
            sleeping_[i] = true;
            scene_.balls[i].previousPosition = scene_.balls[i].position;
            scene_.balls[i].velocity = {0.0, 0.0};
        }
    }
}

std::vector<bool> Simulation::computeSupportedBalls(const std::vector<Contact>& contacts) const {
    std::vector<bool> supported(scene_.balls.size(), false);

    for (const Contact& contact : contacts) {
        if (contact.type == Contact::Type::BallWall && isSupportingNormal(contact.normal)) {
            supported[contact.first] = true;
        }
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (const Contact& contact : contacts) {
            if (contact.type != Contact::Type::BallBall || std::abs(contact.normal.y) < 0.35) {
                continue;
            }

            const std::size_t upper = contact.normal.y > 0.0 ? contact.first : contact.second;
            const std::size_t lower = contact.normal.y > 0.0 ? contact.second : contact.first;
            if (supported[lower] && !supported[upper]) {
                supported[upper] = true;
                changed = true;
            }
        }
    }

    return supported;
}

bool Simulation::isSupportingNormal(const Vec2& normal) const {
    if (std::abs(config_.gravity) <= kEpsilon) {
        return false;
    }
    const double gravityDirection = config_.gravity > 0.0 ? 1.0 : -1.0;
    return normal.y * gravityDirection < -0.35;
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
