#pragma once

#include <string>
#include <utility>

#include "sim/Simulation.h"

namespace sim {

inline void addBoundaryWalls(Scene& scene, double width, double height) {
    scene.walls.push_back({{0.0, 0.0}, {width, 0.0}});
    scene.walls.push_back({{width, 0.0}, {width, height}});
    scene.walls.push_back({{width, height}, {0.0, height}});
    scene.walls.push_back({{0.0, height}, {0.0, 0.0}});
}

inline Scene makeBoxScene(double width, double height, std::string name = "scene") {
    Scene scene;
    scene.name = std::move(name);
    scene.bounds = {0.0, 0.0, width, height};
    addBoundaryWalls(scene, width, height);
    return scene;
}

inline Scene makeInsetBoxScene(double width,
                               double height,
                               double margin,
                               std::string name = "scene") {
    Scene scene;
    scene.name = std::move(name);
    scene.bounds = {margin, margin, width - margin, height - margin};
    addBoundaryWalls(
        scene,
        scene.bounds.maxX - scene.bounds.minX,
        scene.bounds.maxY - scene.bounds.minY);
    for (Wall& wall : scene.walls) {
        wall.a += Vec2{scene.bounds.minX, scene.bounds.minY};
        wall.b += Vec2{scene.bounds.minX, scene.bounds.minY};
    }
    return scene;
}

}  // namespace sim
