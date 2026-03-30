#pragma once

#include <SDL3/SDL.h>

#include "sim/Simulation.h"

namespace sim {

ColorRGBA sampleSurfaceColor(SDL_Surface* surface, int x, int y);
ColorRGBA sampleWorldColor(SDL_Surface* surface, const WorldBounds& bounds, const Vec2& position);
void assignColorsFromFinalPositions(
    Scene& initialScene,
    const Scene& finalScene,
    SDL_Surface* surface);

}  // namespace sim
