#include "sim/ImageSampling.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

ColorRGBA sampleSurfaceColor(SDL_Surface* surface, int x, int y) {
    if (surface == nullptr || surface->w <= 0 || surface->h <= 0) {
        return {};
    }

    x = std::clamp(x, 0, surface->w - 1);
    y = std::clamp(y, 0, surface->h - 1);

    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    std::uint8_t a = 255;
    if (!SDL_ReadSurfacePixel(surface, x, y, &r, &g, &b, &a)) {
        return {};
    }

    return {r, g, b, a};
}

ColorRGBA sampleWorldColor(SDL_Surface* surface, const WorldBounds& bounds, const Vec2& position) {
    if (surface == nullptr || surface->w <= 0 || surface->h <= 0) {
        return {};
    }

    const double width = bounds.maxX - bounds.minX;
    const double height = bounds.maxY - bounds.minY;
    const double normalizedX = width > 0.0 ? (position.x - bounds.minX) / width : 0.0;
    const double normalizedY = height > 0.0 ? (position.y - bounds.minY) / height : 0.0;
    const int pixelX = static_cast<int>(std::lround(
        std::clamp(normalizedX, 0.0, 1.0) * static_cast<double>(surface->w - 1)));
    const int pixelY = static_cast<int>(std::lround(
        std::clamp(normalizedY, 0.0, 1.0) * static_cast<double>(surface->h - 1)));
    return sampleSurfaceColor(surface, pixelX, pixelY);
}

void assignColorsFromFinalPositions(
    Scene& initialScene,
    const Scene& finalScene,
    SDL_Surface* surface) {
    if (initialScene.balls.size() != finalScene.balls.size()) {
        throw std::runtime_error("initial and final scenes must contain the same number of balls");
    }

    for (std::size_t index = 0; index < initialScene.balls.size(); ++index) {
        initialScene.balls[index].color =
            sampleWorldColor(surface, finalScene.bounds, finalScene.balls[index].position);
    }
}

}  // namespace sim
