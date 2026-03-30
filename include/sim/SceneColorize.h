#pragma once

#include <SDL3/SDL.h>

#include "sim/Simulation.h"

namespace sim {

ColorRGBA sampleSceneColor(const Scene& scene, SDL_Surface* surface, const Vec2& worldPosition);
Scene assignColorsFromSettledScene(const Scene& initialScene,
                                   const Scene& settledScene,
                                   SDL_Surface* surface);

}  // namespace sim
