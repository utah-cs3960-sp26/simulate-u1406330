#include "sim/SceneColorize.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "sim/ImageSampling.h"

namespace sim {

namespace {

WorldBounds colorSamplingBounds(const Scene& scene) {
    if (scene.balls.empty()) {
        return scene.bounds;
    }

    WorldBounds bounds;
    bounds.minX = std::numeric_limits<double>::infinity();
    bounds.minY = std::numeric_limits<double>::infinity();
    bounds.maxX = -std::numeric_limits<double>::infinity();
    bounds.maxY = -std::numeric_limits<double>::infinity();

    for (const Ball& ball : scene.balls) {
        bounds.minX = std::min(bounds.minX, ball.position.x - ball.radius);
        bounds.minY = std::min(bounds.minY, ball.position.y - ball.radius);
        bounds.maxX = std::max(bounds.maxX, ball.position.x + ball.radius);
        bounds.maxY = std::max(bounds.maxY, ball.position.y + ball.radius);
    }

    return bounds;
}

}  // namespace

ColorRGBA sampleSceneColor(const Scene& scene, SDL_Surface* surface, const Vec2& worldPosition) {
    if (surface == nullptr) {
        return {};
    }

    const WorldBounds bounds = colorSamplingBounds(scene);
    const double width = bounds.maxX - bounds.minX;
    const double height = bounds.maxY - bounds.minY;
    const double normalizedX =
        width > 0.0 ? (worldPosition.x - bounds.minX) / width : 0.0;
    const double normalizedY =
        height > 0.0 ? (worldPosition.y - bounds.minY) / height : 0.0;

    double sampleX = std::clamp(normalizedX, 0.0, 1.0);
    double sampleY = std::clamp(normalizedY, 0.0, 1.0);
    if (surface->w > 0 && surface->h > 0 && width > 0.0 && height > 0.0) {
        const double worldAspect = width / height;
        const double imageAspect =
            static_cast<double>(surface->w) / static_cast<double>(surface->h);
        if (worldAspect > imageAspect) {
            const double cropHeight = imageAspect / worldAspect;
            sampleY = 0.5 + (sampleY - 0.5) * cropHeight;
        } else if (imageAspect > worldAspect) {
            const double cropWidth = worldAspect / imageAspect;
            sampleX = 0.5 + (sampleX - 0.5) * cropWidth;
        }
    }

    const int pixelX = static_cast<int>(std::lround(
        std::clamp(sampleX, 0.0, 1.0) * static_cast<double>(surface->w - 1)));
    const int pixelY = static_cast<int>(std::lround(
        std::clamp(sampleY, 0.0, 1.0) * static_cast<double>(surface->h - 1)));
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
