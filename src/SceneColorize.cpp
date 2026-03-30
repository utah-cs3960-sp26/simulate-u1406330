#include "sim/SceneColorize.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "sim/ImageSampling.h"

namespace sim {

ColorRGBA sampleSceneColor(const Scene& scene, SDL_Surface* surface, const Vec2& worldPosition) {
    if (surface == nullptr) {
        return {};
    }

    const double width = scene.bounds.maxX - scene.bounds.minX;
    const double height = scene.bounds.maxY - scene.bounds.minY;
    const double normalizedX =
        width > 0.0 ? (worldPosition.x - scene.bounds.minX) / width : 0.0;
    const double normalizedY =
        height > 0.0 ? (worldPosition.y - scene.bounds.minY) / height : 0.0;
    const int pixelX = static_cast<int>(std::lround(
        std::clamp(normalizedX, 0.0, 1.0) * static_cast<double>(surface->w - 1)));
    const int pixelY = static_cast<int>(std::lround(
        std::clamp(normalizedY, 0.0, 1.0) * static_cast<double>(surface->h - 1)));
    return sampleSurfaceColor(surface, pixelX, pixelY);
}

Scene assignColorsFromSettledScene(const Scene& initialScene,
                                   const Scene& settledScene,
                                   SDL_Surface* surface) {
    if (initialScene.balls.size() != settledScene.balls.size()) {
        throw std::invalid_argument("initial and settled scenes must contain the same number of balls");
    }

    Scene output = initialScene;
    for (std::size_t i = 0; i < output.balls.size(); ++i) {
        output.balls[i].color = sampleSceneColor(settledScene, surface, settledScene.balls[i].position);
    }
    return output;
}

}  // namespace sim
