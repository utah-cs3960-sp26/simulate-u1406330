#include <cstdlib>
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

bool fillPixel(SDL_Surface* surface, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    SDL_Rect rect{x, y, 1, 1};
    const Uint32 pixel = SDL_MapSurfaceRGBA(surface, r, g, b, 255);
    return SDL_FillSurfaceRect(surface, &rect, pixel);
}

bool testImageSampling() {
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
        SDL_CreateSurface(2, 2, SDL_PIXELFORMAT_RGBA32),
        &SDL_DestroySurface);
    if (!expect(surface != nullptr, "failed to create temporary test surface")) {
        return false;
    }

    if (!expect(fillPixel(surface.get(), 0, 0, 255, 0, 0), "failed to fill top-left pixel") ||
        !expect(fillPixel(surface.get(), 1, 0, 0, 255, 0), "failed to fill top-right pixel") ||
        !expect(fillPixel(surface.get(), 0, 1, 0, 0, 255), "failed to fill bottom-left pixel") ||
        !expect(fillPixel(surface.get(), 1, 1, 255, 255, 0), "failed to fill bottom-right pixel")) {
        return false;
    }

    sim::Scene scene = sim::makeBoxScene(2.0, 2.0, "sample");
    const sim::ColorRGBA topLeft = sim::sampleSceneColor(scene, surface.get(), {0.0, 0.0});
    const sim::ColorRGBA topRight = sim::sampleSceneColor(scene, surface.get(), {2.0, 0.0});
    const sim::ColorRGBA bottomLeft = sim::sampleSceneColor(scene, surface.get(), {0.0, 2.0});
    const sim::ColorRGBA bottomRight = sim::sampleSceneColor(scene, surface.get(), {2.0, 2.0});

    return expect(topLeft.r == 255 && topLeft.g == 0 && topLeft.b == 0, "top-left color mismatch") &&
           expect(topRight.r == 0 && topRight.g == 255 && topRight.b == 0, "top-right color mismatch") &&
           expect(bottomLeft.r == 0 && bottomLeft.g == 0 && bottomLeft.b == 255, "bottom-left color mismatch") &&
           expect(bottomRight.r == 255 && bottomRight.g == 255 && bottomRight.b == 0, "bottom-right color mismatch");
}

bool testSceneColorizationPipeline() {
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
        SDL_CreateSurface(2, 2, SDL_PIXELFORMAT_RGBA32),
        &SDL_DestroySurface);
    if (!expect(surface != nullptr, "failed to create colorization surface")) {
        return false;
    }

    if (!expect(fillPixel(surface.get(), 0, 0, 255, 0, 0), "failed to fill top-left pixel") ||
        !expect(fillPixel(surface.get(), 1, 0, 0, 255, 0), "failed to fill top-right pixel") ||
        !expect(fillPixel(surface.get(), 0, 1, 0, 0, 255), "failed to fill bottom-left pixel") ||
        !expect(fillPixel(surface.get(), 1, 1, 255, 255, 0), "failed to fill bottom-right pixel")) {
        return false;
    }

    sim::Scene initial = sim::makeBoxScene(2.0, 2.0, "initial");
    sim::Scene settled = initial;

    sim::Ball first;
    first.position = {0.25, 0.25};
    first.radius = 0.1;
    first.inverseMass = 1.0;
    initial.balls.push_back(first);

    sim::Ball second = first;
    second.position = {1.75, 0.25};
    initial.balls.push_back(second);

    settled.balls = initial.balls;
    settled.balls[0].position = {1.75, 1.75};
    settled.balls[1].position = {0.25, 1.75};

    const sim::Scene recolored = sim::assignColorsFromSettledScene(initial, settled, surface.get());
    return expect(recolored.balls[0].position.x == initial.balls[0].position.x &&
                      recolored.balls[0].position.y == initial.balls[0].position.y,
                  "recolored scene should keep initial positions") &&
           expect(recolored.balls[1].position.x == initial.balls[1].position.x &&
                      recolored.balls[1].position.y == initial.balls[1].position.y,
                  "recolored scene should keep initial positions") &&
           expect(recolored.balls[0].color.r == 255 &&
                      recolored.balls[0].color.g == 255 &&
                      recolored.balls[0].color.b == 0,
                  "first ball should use settled bottom-right color") &&
           expect(recolored.balls[1].color.r == 0 &&
                      recolored.balls[1].color.g == 0 &&
                      recolored.balls[1].color.b == 255,
                  "second ball should use settled bottom-left color");
}

}  // namespace

int main() {
    if (!testImageSampling()) {
        return EXIT_FAILURE;
    }
    if (!testSceneColorizationPipeline()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
