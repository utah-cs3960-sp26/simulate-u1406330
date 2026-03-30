# simulate-u1406330

Deterministic 2D ball physics simulator in C++ with an SDL3 visualizer and a headless debugging harness.

## What it includes

- `simulator` executable with SDL3 visual mode and deterministic headless mode
- `scene_colorize` executable that settles a CSV scene and samples colors from an image
- circular balls, immovable wall segments, gravity, and configurable restitution
- adaptive substeps plus positional projection to prevent tunneling and persistent overlap
- uniform-grid broadphase for large ball counts
- renderer and simulation-path optimizations aimed at keeping the thousand-ball scene responsive
- seeded scenarios for repeatable debugging and regression checks

## Build

The project uses CMake and expects SDL3 to be installed and discoverable by `find_package(SDL3 CONFIG REQUIRED)`.

Windows:

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

macOS:

```bash
xcode-select --install
brew install cmake sdl3
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

If CMake cannot find SDL3 on macOS, configure with Homebrew's prefix explicitly:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$(brew --prefix sdl3)"
```

## Run

```powershell
build\simulator.exe
build\simulator.exe --mode headless --scenario container --steps 1200 --dump-every 120
```

Useful flags:

- `--restitution 0.35`
- `--balls 1000`
- `--seed 7`
- `--dt 0.0166667`
- `--gravity 1400`
- `--scenario container|stack|gap`
- `--scene-csv path\\to\\initial.csv`
- `--dump-final`
- `--dump-final-csv path\\to\\final.csv`

CSV scene files use one row per ball with `x,y,r,g,b,radius`. The simulator loads that format as the ball list for the selected scenario and writes the same format back out with the final ball positions.

The image colorization tool uses the same CSV format:

```powershell
build\\scene_colorize.exe --scene-csv path\\to\\initial.csv --image path\\to\\image.png --output-csv path\\to\\colored.csv
```

## Debugging loop

Use headless mode first. It prints deterministic summaries including contact counts, overlap count, max penetration, total kinetic energy, max speed, and escaped-ball count.

Examples:

```powershell
build\simulator.exe --mode headless --scenario gap --steps 600 --dump-every 30
build\simulator.exe --mode headless --scenario stack --restitution 0.05 --dump-final
```

`container` is the main thousand-ball scene used by the SDL app. `stack` is a compact settling case for regression checks. `gap` is a wall-tightness case intended to catch phasing through narrow openings.

## Performance notes

- The visualizer was tuned to stay responsive with the main thousand-ball container scene.
- Ball rendering uses scanline fills instead of per-pixel point drawing to cut SDL draw overhead.
- The broadphase uses a dense grid so collision candidate generation stays cheaper as ball counts rise.
- Velocity clamping and less aggressive default substep settings keep frame cost from exploding when contacts get noisy.

## Design notes

- Restitution changes bounce response, not the overlap-removal solver, so lower restitution settles faster while the final packed footprint remains effectively unchanged.
- Low-speed restitution is disabled to suppress endless micro-bouncing.
- The simulation uses a fixed step with internal substeps chosen from current velocity and ball radius, then clamps runaway speeds so the sim stays tractable in real time.
