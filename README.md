# Mech IK Core (GaitCore)

A standalone, multithreaded procedural inverse kinematics and locomotion engine for legged mechs (`2`, `4`, `6`, and `8` legs), paired with an optional 3D C++ showcase and benchmark application.

Built to be dropped directly into an existing game loop: you push 12 control floats per mech, answer read-only world raycasts and sphere-pushes on your own geometry, and pull solved world joint positions plus locomotion events each frame.

## Key Features

- **Zero-dependency C11 core (`IK/c/`)**: Planar 2-bone analytic + active-set DLS / FABRIK IK solver, rhythm + reactive gait scheduler, situation-aware gait selection & smooth transitions, swing arc shaping, foothold edge/clearance filtering, wall-climbing, mantling, jumping/bracing, sliding, and dynamic CoM balance.
- **Deterministic worker pool**: Steps hundreds to 1,000+ mechs in parallel across CPU cores (`gc_thread.c`) while draining events in handle order for identical output regardless of thread count (`1T == 4T == 8T`).
- **Custom mech archetypes**: Define 2-, 4-, 6-, or 8-legged custom rigs at runtime via `GcMechDesc` and `gc_register_mech_type()` without editing library source, or spawn any of the 9 built-in presets (`LYNX` through `COLOSSUS`).
- **Zero-dependency Java 17+ core (`IK/java/`)**: Direct 1:1 multithreaded port (`gaitcore`) using only `java.base`, with the same custom archetype API (`GcMechDesc`), `GcStubWorld`, and `1T == 4T == 8T` bit-exact determinism via persistent daemon worker threads (`ThreadPool.java`).
- **Reference Python core (`IK/python/`)**: Matching Python twin (`gaitcore`) with 53 unit tests, offline regression/film tools, and optional live `F5` hot-reload in the demo app.
- **Copy-only portability**: `IK/c/`, `IK/java/` (or the full `IK/` directory) have zero dependencies on the demo app, renderer, ImGui, or sokol and drop cleanly into external engines.

---

## Repository Layout

```text
.
├── CMakeLists.txt               # Single-project CMake build (core + tests + example + optional demo)
├── CMakePresets.json            # Windows / Linux / macOS CMake configure, build & test presets
├── README.md
├── IK/                          # <-- Self-contained IK core (copy just IK/c/ or IK/ into your game)
│   ├── c/                       # Standalone C11 library (CMake target: gaitcore::c)
│   │   ├── CMakeLists.txt
│   │   ├── include/gaitcore.h   # Single public C/C++ header
│   │   ├── src/                 # Multithreaded engine & Win32/POSIX thread pool
│   │   └── tests/               # C11 unit, determinism & multithread tests
│   ├── java/                    # Standalone Java 17+ library (CMake target: gaitcore_java)
│   │   ├── CMakeLists.txt
│   │   ├── src/main/java/gaitcore/ # Pure java.base multithreaded engine (zero external deps)
│   │   └── src/test/java/gaitcore/ # Java unit, determinism & custom-mech tests
│   └── python/                  # Standalone Python reference & diagnostic tooling
│       ├── gaitcore/            # Python gaitcore package
│       ├── tests/               # 53 unit tests
│       └── tools/               # Headless sim, limit/fuzz/foothold/glitch/gait checks & film
├── examples/
│   ├── CMakeLists.txt
│   ├── custom_mech_example.c    # Minimal C11 world + custom mech integration example
│   └── CustomMechExample.java   # Minimal Java 17+ world + custom mech integration example
├── demo/                        # Optional interactive sokol + Dear ImGui 3D showcase
│   ├── CMakeLists.txt
│   ├── extern/                  # Vendored sokol & stb_image_write
│   ├── framework/               # Sample host, PBR renderer, camera, ImGui shell
│   └── scenes/                  # Mech Sandbox, Frontline Ruins, Gait Lab, IK Benchmark (up to 1K mechs)
└── docs/
    ├── INTEGRATION.md           # Step-by-step guide to integrating IK/ into a game engine
    ├── API_REFERENCE.md         # Public C API, custom descriptors & packed stream layouts
    └── ARCHITECTURE.md          # Solvers, gait state machine, foothold selection & balance
```

---

## Building with CMake

### Requirements

- **Core library, tests, and C example (`IK/c/`, `examples/`)**:
  - CMake `3.22+`
  - A C11 compiler (MSVC / Visual Studio 2022+, Clang, or GCC; Win32 on Windows, `pthreads` + `libm` on Linux/macOS)
- **Java library, tests, and Java example (`GAITCORE_BUILD_JAVA=ON`)**:
  - Java `17+` JDK (`javac` / `java`, zero external Maven/Gradle dependencies)
- **Optional 3D Demo (`GAITCORE_BUILD_DEMO=ON`)**:
  - C++20 compiler
  - Network access on first configure to fetch Dear ImGui (`v1.92.7`) and ImPlot (`v0.17`) into `.fetchcontent-cache/`
  - **Linux only**: OpenGL + X11 development headers (`libgl-dev libx11-dev libxi-dev libxcursor-dev`)
  - **Optional Python backend**: 64-bit Python `3.10+` with development headers (`include/` + `libs/`)

### Full Single-Project Build (Library + Tests + Example + 3D Demo)

```bash
# Configure and build (Release)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel

# Run C unit, custom-mech, and multithread determinism tests
ctest --test-dir build -C Release --output-on-failure
```

Multi-config generators (Visual Studio / Xcode) ignore `-DCMAKE_BUILD_TYPE` and select the configuration with `--config Release` (or `Debug`) during `cmake --build` and `-C Release` during `ctest`.

Using CMake Presets:

```bash
# Windows
cmake --preset windows
cmake --build --preset windows-release
ctest --preset windows-release

# Linux
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release

# macOS
cmake --preset macos
cmake --build --preset macos-release
ctest --preset macos-release
```

### Library-Only Build (Skip 3D Demo & ImGui/sokol Dependencies)

```bash
cmake -S . -B build-lib -DCMAKE_BUILD_TYPE=Release -DGAITCORE_BUILD_DEMO=OFF
cmake --build build-lib --config Release --parallel
ctest --test-dir build-lib -C Release --output-on-failure
```

Or with presets: `cmake --preset windows-lib-only && cmake --build --preset windows-lib-only` (`linux-lib-only` / `macos-lib-only` on Unix).

### Standalone `IK/c` Build (Zero Parent-Repo Files Needed)

To verify or build `IK/c/` completely in isolation (for example, after copying `IK/c/` into another game):

```bash
cmake -S IK/c -B build-c -DCMAKE_BUILD_TYPE=Release
cmake --build build-c --config Release --parallel
ctest --test-dir build-c -C Release --output-on-failure
```

### Standalone `IK/java` Build (Zero External Dependencies)

Build and test `IK/java/` standalone with CMake:

```bash
cmake -S IK/java -B build-java
cmake --build build-java --config Release
ctest --test-dir build-java -C Release --output-on-failure
```

Or compile and run directly with `javac` / `java` (Java 17+):

```powershell
mkdir out-java
javac --release 17 -Xlint:all -d out-java (Get-ChildItem IK/java/src/main/java/gaitcore/*.java)
javac --release 17 -Xlint:all -cp out-java -d out-java IK/java/src/test/java/gaitcore/GaitCoreTest.java examples/CustomMechExample.java
java -cp out-java gaitcore.GaitCoreTest
java -cp out-java CustomMechExample
```

### CMake Options Reference

| Option | Default | Description |
|---|---|---|
| `GAITCORE_BUILD_TESTS` | `ON` when top-level | Build `gaitcore_c_tests` and `gaitcore_java_tests` and register with CTest |
| `GAITCORE_BUILD_EXAMPLES` | `ON` when top-level | Build `custom_mech_example` and `custom_mech_example_java` |
| `GAITCORE_BUILD_JAVA` | `ON` when top-level | Build Java 17+ `gaitcore_java` (`gaitcore.jar`) and Java tests |
| `GAITCORE_BUILD_DEMO` | `ON` when top-level | Build the interactive 3D demo (`mech_ik_demo`) |
| `GAITCORE_ENABLE_PYTHON` | `ON` | Embed Python 3.10+ in `mech_ik_demo` when development libs are present |
| `APP_BUILD_SHADERS` | `OFF` | Recompile `.glsl` shaders with `sokol-shdc` instead of using checked-in headers |

All `GAITCORE_BUILD_*` defaults evaluate to `OFF` automatically when included via `add_subdirectory(...)` inside another project, so external games only build the `gaitcore::c` target by default.

---

## Using the IK Core in Another Mech Game

### 1. Copy `IK/c/` and Link `gaitcore::c`

Copy `IK/c/` into `third_party/gaitcore/` (no other folders are required) and add to your `CMakeLists.txt`:

```cmake
add_subdirectory(third_party/gaitcore)
target_link_libraries(my_mech_game PRIVATE gaitcore::c)
```

Include the single public header:

```c
#include "gaitcore.h"
```

### 2. Minimal Integration Sketch

```c
GcContext* ctx = gc_create(0); // 0 = hardware thread count
GcWorldCallbacks cb = { my_raycast, my_sphere_push, my_terrain_height, &my_world };
gc_set_world(ctx, &cb);

// Register a custom mech (or spawn preset 0..8 directly with gc_spawn)
GcMechDesc desc;
gc_mech_desc_init_default(&desc, 4); // 2=biped, 4=quadruped, 6=hexapod, 8=octopod
snprintf(desc.name, sizeof(desc.name), "PANTHER_MK2");
desc.speed_walk = 18.0f;
desc.speed_run  = 28.0f;
int custom_kind = gc_register_mech_type(ctx, &desc);
int mech = gc_spawn(ctx, custom_kind, 0.0f, 0.0f, 0.0f, 0.0f);

// Per frame: push packed inputs -> step in parallel -> read body + leg joint transforms
float inputs[GC_INPUT_STRIDE] = {0};
inputs[GC_IN_FWD] = 1.0f;
inputs[GC_IN_RUN] = 1.0f;
gc_step(ctx, dt, inputs, 1);

float poses[256];
int nfloats = gc_poses(ctx, poses, 256);
// poses[GC_POSE_POS + 0..2], poses[GC_POSE_RIGHT/UP/FWD], and per-leg joint positions follow.
```

See [examples/custom_mech_example.c](examples/custom_mech_example.c) and [examples/CustomMechExample.java](examples/CustomMechExample.java) for complete runnable programs, [docs/INTEGRATION.md](docs/INTEGRATION.md) for the integration walkthrough, and [docs/API_REFERENCE.md](docs/API_REFERENCE.md) for full struct and binary-stream details.

---

## Running Examples, Tests & 3D Demo

Run from your build directory or repo root:

```powershell
# Minimal C example
.\build\bin\Release\custom_mech_example.exe

# Minimal Java example (from CMake-built jars)
java -cp "build/examples/custom_mech_example_java.jar;build/IK/java/gaitcore.jar" CustomMechExample

# Interactive 3D demo
.\build\bin\Release\mech_ik_demo.exe
```

### Demo Scenes (`mech_ik_demo`)

- **`Mechs/Frontline Ruins`**: Cratered battlefield, trench duckboard bridges, Dragon's Teeth anti-tank obstacles, pillboxes, breached 8 m fortress wall, watchtower, and timed assault course with AI walkers.
- **`Mechs/Mech Sandbox`**: Hub playground with ledges, slopes, crouch tunnel, climbing walls, four obstacle courses (`Agility`, `Vertical`, `Slide`, `Hills`), and autopilot.
- **`Mechs/Gait Lab`**: Straight 400 m test lane (`Flat`, `Stairs`, `Rubble`, `Slopes`, `Ledges`) with side camera and real-time Hildebrand footfall diagram.
- **`Mechs/IK Benchmark`**: Crowd stress test (1–1,000 mechs, `Flat` / `~5,072`-obstacle `Rubble` / `Frontline`, `500` & `1K` presets, decoupled IK/probe frequency sliders, distance LOD toggle, and `M`-key live stage/worker profiler).

### Demo Controls & CLI

| Input | Action |
|---|---|
| `W` `S` / `A` `D` | Move / strafe relative to camera |
| Right-drag or `L` | Aim camera (mech turns toward aim; pitches up/down in air) |
| `Shift` / `Ctrl` | Run / slow walk |
| `C` / `Space` / `X` | Crouch toggle / jump / quick slide |
| `Z` / `G` / `K` | Climb toggle (climbers) / force next gait / balance shove |
| `1`–`9` / `N` | Switch mech archetype / cycle driven mech |
| `H` / `V` / `M` | IK debug overlay / camera mode / profiler drawer |
| `F5` / `R` | Hot-reload Python core (Python backend) / restart scene |

Useful environment overrides & CLI flags:
- `--list`, `--sample "Mechs/IK Benchmark"`, `--frames N`, `--screenshot out.png`, `--hide-ui`, `--pause`, `--size WxH`
- `MECH_BACKEND=c|python`, `MECH_THREADS=N`, `MECH_BENCH_COUNT=1..1000`, `MECH_BENCH_TERRAIN=0|1|2`, `MECH_BENCH_PROFILER=1`

### Python Reference Suite (`IK/python/`)

```bash
python -m unittest discover -s IK/python/tests -v
python IK/python/tools/headless_sim.py
python IK/python/tools/limit_check.py
python IK/python/tools/fuzz_check.py
python IK/python/tools/foothold_check.py
python IK/python/tools/glitch_check.py
python IK/python/tools/gait_select_check.py
```

---

## Docs

- **[docs/INTEGRATION.md](docs/INTEGRATION.md)** — World callbacks, coordinate conventions, custom mech descriptors (`2`/`4`/`6`/`8` legs), skeletal pose mapping, threading & LOD config.
- **[docs/API_REFERENCE.md](docs/API_REFERENCE.md)** — Full C API (`gaitcore.h`), Python API (`gaitcore.api`), packed float layouts, and built-in presets.
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — Planar IK solvers, rhythm+reactive gait scheduler, situation-aware gait selection, foothold scoring, balance, climbing, jumping, and sliding design notes.

---

## Licenses & Attribution

- **`IK/` procedural gait & IK core** and `examples/` — project source.
- **3D demo shell & renderer (`demo/framework/`)** — derived from Erin Catto's Box3D sample framework (MIT, [demo/framework/LICENSE](demo/framework/LICENSE)).
- **Vendored libraries (`demo/extern/`)** — [sokol](demo/extern/sokol/LICENSE) (zlib/libpng), `stb_image_write.h` (public domain / MIT), plus Dear ImGui & ImPlot (MIT, fetched when `GAITCORE_BUILD_DEMO=ON`).
