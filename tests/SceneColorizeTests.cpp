#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>

#include <SDL3/SDL.h>

#include "sim/SceneColorize.h"
#include "sim/SceneSetup.h"

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool testRecolorSceneFromFinalPositions() {
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
        SDL_CreateSurface(2, 2, SDL_PIXELFORMAT_RGBA32),
        &SDL_DestroySurface);
    if (!expect(surface != nullptr, "failed to create test surface")) {
        return false;
    }

    auto fillPixel = [&](int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
        SDL_Rect rect{x, y, 1, 1};
        const Uint32 pixel = SDL_MapSurfaceRGBA(surface.get(), r, g, b, 255);
        return SDL_FillSurfaceRect(surface.get(), &rect, pixel);
    };
    if (!expect(fillPixel(0, 0, 255, 0, 0), "failed to fill top-left pixel") ||
        !expect(fillPixel(1, 0, 0, 255, 0), "failed to fill top-right pixel") ||
        !expect(fillPixel(0, 1, 0, 0, 255), "failed to fill bottom-left pixel") ||
        !expect(fillPixel(1, 1, 255, 255, 0), "failed to fill bottom-right pixel")) {
        return false;
    }

    sim::Scene initialScene = sim::makeBoxScene(2.0, 2.0, "initial");
    sim::Ball first;
    first.position = {0.25, 0.25};
    first.radius = 0.25;
    initialScene.balls.push_back(first);

    sim::Ball second;
    second.position = {1.75, 1.75};
    second.radius = 0.25;
    initialScene.balls.push_back(second);

    sim::Scene settledScene = initialScene;
    settledScene.balls[0].position = {0.1, 0.1};
    settledScene.balls[1].position = {1.1, 1.1};

    const sim::Scene recoloredScene =
        sim::assignColorsFromSettledScene(initialScene, settledScene, surface.get());

    return expect(recoloredScene.balls[0].color.r == 255 &&
                      recoloredScene.balls[0].color.g == 0 &&
                      recoloredScene.balls[0].color.b == 0,
                  "top-left color mismatch") &&
           expect(recoloredScene.balls[1].color.r == 255 &&
                      recoloredScene.balls[1].color.g == 255 &&
                      recoloredScene.balls[1].color.b == 0,
                  "bottom-right color mismatch") &&
           expect(recoloredScene.balls[0].position.x == 0.25 &&
                      recoloredScene.balls[1].position.x == 1.75,
                  "recoloring changed ball positions");
}

bool testRecolorUsesOccupiedSettledBoundsInsteadOfFullSceneBounds() {
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
        SDL_CreateSurface(2, 2, SDL_PIXELFORMAT_RGBA32),
        &SDL_DestroySurface);
    if (!expect(surface != nullptr, "failed to create occupied-bounds test surface")) {
        return false;
    }

    auto fillPixel = [&](int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
        SDL_Rect rect{x, y, 1, 1};
        const Uint32 pixel = SDL_MapSurfaceRGBA(surface.get(), r, g, b, 255);
        return SDL_FillSurfaceRect(surface.get(), &rect, pixel);
    };
    if (!expect(fillPixel(0, 0, 255, 0, 0), "failed to fill top-left pixel") ||
        !expect(fillPixel(1, 0, 0, 255, 0), "failed to fill top-right pixel") ||
        !expect(fillPixel(0, 1, 0, 0, 255), "failed to fill bottom-left pixel") ||
        !expect(fillPixel(1, 1, 255, 255, 0), "failed to fill bottom-right pixel")) {
        return false;
    }

    sim::Scene initialScene = sim::makeBoxScene(10.0, 10.0, "initial");
    sim::Ball first;
    first.position = {2.0, 2.0};
    first.radius = 0.5;
    initialScene.balls.push_back(first);

    sim::Ball second;
    second.position = {8.0, 8.0};
    second.radius = 0.5;
    initialScene.balls.push_back(second);

    sim::Scene settledScene = initialScene;
    settledScene.balls[0].position = {4.5, 4.5};
    settledScene.balls[1].position = {5.5, 5.5};

    const sim::Scene recoloredScene =
        sim::assignColorsFromSettledScene(initialScene, settledScene, surface.get());

    return expect(recoloredScene.balls[0].color.r == 255 &&
                      recoloredScene.balls[0].color.g == 0 &&
                      recoloredScene.balls[0].color.b == 0,
                  "occupied bounds should map first ball to top-left") &&
           expect(recoloredScene.balls[1].color.r == 255 &&
                      recoloredScene.balls[1].color.g == 255 &&
                      recoloredScene.balls[1].color.b == 0,
                  "occupied bounds should map second ball to bottom-right");
}

}  // namespace

int main() {
    return testRecolorSceneFromFinalPositions() &&
                   testRecolorUsesOccupiedSettledBoundsInsteadOfFullSceneBounds()
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
