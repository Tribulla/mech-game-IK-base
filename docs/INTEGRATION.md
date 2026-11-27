# Integrating `IK/` into a Mech Game Engine

This guide covers everything required to drop the **gaitcore** IK and procedural locomotion engine into your own game engine or simulation project.

---

## 1. Integration Options

### Option A — Copy `IK/c/` into Your Repo (Recommended for Games)

`IK/c/` has **zero external dependencies** outside the C11 standard library (`<math.h>`, `<stdlib.h>`, `<string.h>`, `<stdint.h>`) and native OS threads (`Windows.h` / `pthreads`). It does **not** need `demo/`, `IK/python/`, ImGui, or sokol.

1. Copy the `IK/c/` directory into your project (e.g. `third_party/gaitcore/`).
2. Add it to your build:
   ```cmake
   add_subdirectory(third_party/gaitcore)
   target_link_libraries(my_game PRIVATE gaitcore::c)
   ```
   *(When included as a subproject, `GAITCORE_BUILD_TESTS` defaults to `OFF`, so only the static `gaitcore_c` library target is built.)*
3. Include `"gaitcore.h"`.

### Option B — Direct Source Inclusion (No CMake)

Compile all `.c` files under `IK/c/src/` as C11 with `IK/c/include/` and `IK/c/src/` on your include path:
- Windows: zero extra libraries needed beyond standard CRT + Win32.
- Linux/macOS: link `-lpthread -lm`.

### Option C — Copy `IK/java/` for Java 17+ / JVM Engines

`IK/java/` has **zero external dependencies** outside `java.base`. Copy `IK/java/src/main/java/gaitcore/` into your source tree (or include `IK/java/` via CMake `add_subdirectory` to link `gaitcore.jar`).

```java
try (GcContext ctx = GcContext.create(0)) { // 0 = auto thread count
    ctx.setWorld(myWorldCallbacks);        // implements GcWorldCallbacks (read-only & thread-safe)

    GcMechDesc desc = GcMechDesc.createDefault(4);
    desc.name = "PANTHER_MK2";
    desc.speedWalk = 18.0f;
    desc.speedRun  = 28.0f;
    int kind = ctx.registerMechType(desc);
    int mech = ctx.spawn(kind, 0.0f, 0.0f, 0.0f, 0.0f);

    float[] inputs = new float[GaitCore.INPUT_STRIDE];
    float[] poses  = new float[ctx.floatsPerMech(mech)];
    inputs[GaitCore.IN_FWD] = 1.0f;
    ctx.step(1.0f / 60.0f, inputs, 1);
    ctx.poses(poses, poses.length);
}
```

See [../examples/CustomMechExample.java](../examples/CustomMechExample.java) for a full runnable Java integration program.

### Option D — Reference / Prototyping in Python (`IK/python/`)

Copy `IK/python/gaitcore/` anywhere on your Python `sys.path`. Implement a matching world object (`world.set_world(...)`) exposing `raycast`, `sphere_push`, and `terrain_height`, or run embedded CPython via `gaitcore.api` just like [../demo/scenes/mech_bridge.h](../demo/scenes/mech_bridge.h).

---

## 2. Coordinate Space & Units

All positions, lengths, and velocities use **SI meters / seconds**, right-handed, **`+Y` up**, **`+Z` forward** at yaw `0`:

| Axis | Body-Local Meaning | Notes |
|---|---|---|
| `+Y` | Up | Dorsal / roof normal |
| `+Z` | Forward | Heading direction at `yaw = 0` (`(+sin(yaw), 0, +cos(yaw))`) |
| `right` | `fwd x up` | Note the cross-product order: at `yaw = 0`, `fwd(0,0,1) x up(0,1,0) = (-1, 0, 0)` in world space. |

Why this matters when placing leg mounts (`mount[0]`, `rest_xz[0]`):
- In `gaitcore`'s body basis (`frame.right = fwd x up`), **`mount[0] < 0`** sits on the `-right` side (pointing toward world `+X` at `yaw = 0`, assigned side `-1`), and **`mount[0] > 0`** sits on the `+right` side (pointing toward world `-X`, assigned side `+1`).
- For symmetrical rigs, mirror left/right pairs strictly with opposite signs on `mount[0]` and `rest_xz[0]`. When converting `GC_POSE_*` back into a standard `+X right, +Y up, +Z forward` render matrix, simply negate the exported `right` column: `[ -right | up | fwd ]` (determinant `+1`).

> **Adapting to `+Z`-up or `-Z`-forward engines (Unreal, Unity, Godot, Roblox)**: convert transforms cleanly at the wrapper boundary (world raycast/sphere-push inputs and pose matrix extraction) rather than altering internal gait math.

---

## 3. Implementing Thread-Safe World Queries (`GcWorldCallbacks`)

`gaitcore` owns **no level geometry**. During `gc_step()`, worker threads step each mech concurrently and invoke your `GcWorldCallbacks` **with no locks held inside `gaitcore`**:

```c
typedef struct GcWorldCallbacks {
    GcRaycastFn       raycast;
    GcSpherePushFn    sphere_push;
    GcTerrainHeightFn terrain_height;
    void*             user;
} GcWorldCallbacks;
```

### Callback Contract (Critical for Multithreading)

Your callbacks **must be thread-safe and strictly read-only** during `gc_step()`:
- **`raycast(user, ox, oy, oz, dx, dy, dz, max_dist, out_hit)`**:
  - `(dx, dy, dz)` is a unit direction vector.
  - Ignore hits where the ray origin starts inside solid geometry (`t <= 0`) so interior terrain probes don't trap feet underground.
  - Return `1` and fill `out_hit->t`, `px, py, pz`, `nx, ny, nz` (unit surface normal), and `surface` ID if hit within `[0, max_dist]`; otherwise return `0`.
  - Keep raycasts against mech hulls themselves **filtered out** (collide only against static world/terrain/props, not the querying mech's own collider!).
- **`sphere_push(user, cx, cy, cz, radius, out_xyz, out_nxyz)`**:
  - Test sphere `(cx, cy, cz, radius)` against world colliders and resolve overlap onto `out_xyz[3]` + unit push normal `out_nxyz[3]`.
  - Return `1` if pushed, `0` otherwise (still copy `(cx, cy, cz)` into `out_xyz` when returning `0`).
  - Used for hull belly/capsule clearance, knee/foot rock extraction, and slide impacts.
- **`terrain_height(user, x, z)`**:
  - Fast height query at `(x, z)` for heightfields (may simply return flat ground height if your world only uses meshes/colliders in `raycast`, or NULL if not needed).

> **Performance tip**: If your world contains hundreds of discrete rocks or prop boxes, index them with a read-only broadphase (BVH, octree, or 2D grid — see `World::BuildGrid` in [../demo/scenes/mech_world.h](../demo/scenes/mech_world.h)) before calling `gc_step()`. Fast queries keep 1,000 mechs in the low-millisecond range on multicore CPUs.

---

## 4. Defining Custom Mechs (`GcMechDesc`)

You can spawn built-in presets (`0..gc_kind_count()-1`) or define arbitrary `2`-, `4`-, `6`-, or `8`-legged machines:

```c
GcMechDesc d;
gc_mech_desc_init_default(&d, 4); // Start from valid 2-, 4-, 6-, or 8-leg defaults
snprintf(d.name, sizeof(d.name), "HEAVY_SPIDER");

// Geometry & IK plane
d.plane_mode = GC_LEG_PLANE_RADIAL; // Radial coxa yaw (spiders) or GC_LEG_PLANE_SAGITTAL (bipeds / narrow tanks)
d.coxa = 0.30f;
d.num_segments = 3;                 // 2..4 segments (excluding coxa)
d.segment_lengths[0] = 1.60f;
d.segment_lengths[1] = 1.50f;
d.segment_lengths[2] = 0.55f;
d.bend_sides[0] = 1;                // knee folds upward/outward
d.bend_sides[1] = 1;                // ankle folds in same direction

// Body capsule bounds, ride height, locomotion
d.body_half[0] = 0.90f; d.body_half[1] = 0.50f; d.body_half[2] = 1.45f;
d.ride_height  = 1.45f;
d.speed_slow   = 10.0f;
d.speed_walk   = 24.0f;
d.speed_run    = 36.0f;
d.jump_speed   = 15.0f;
d.climber      = 0;                 // set to 1 to allow wall/ceiling climbing

int my_kind = gc_register_mech_type(ctx, &d);
int handle  = gc_spawn(ctx, my_kind, x, y, z, yaw);
```

### Authoring Rules for Reliable Rigs

1. **Leg ordering matters for gaits**:
   List legs in front-to-back left/right pairs (`[L0, R0, L1, R1, ...]`), where `L*` sits on one side (`side = -1`) and `R*` on the opposite (`side = +1`). Built-in quadruped, hexapod, and octopod phase tables pair indices `(0,1)`, `(2,3)`, `(4,5)`, `(6,7)`.
2. **Rest reach ratio (`50–65%` of total leg reach)**:
   Compute the distance from `legs[i].mount` to `(legs[i].rest_xz[0], -ride_height, legs[i].rest_xz[1])`. Keep that distance around `50–65%` of `(coxa + sum(segment_lengths))` so legs have comfortable travel both inward and outward for uneven terrain and bounding strides.
3. **`plane_mode`**:
   - Use `GC_LEG_PLANE_RADIAL` (`0`) for radial spider/hexapod/octopod legs where a coxa turret yaws around hull up toward the foothold. Set `coxa > 0` (e.g. `0.2–0.45 m`) and `limits.has_coxa = 1`.
   - Use `GC_LEG_PLANE_SAGITTAL` (`1`) for bipeds (`STRIDER`) or under-hull narrow walkers (`BASTION`), with `coxa = 0.0f` and `limits.has_roll = 1`.
4. **3-segment spider legs vs general N-segment legs**:
   With `num_segments == 3`, `bend_sides = {1, 1}` solves via fast closed-form law of cosines (shared knee/ankle bend); `{1, -1}` or `num_segments == 4` runs bounded 2D FABRIK + active-set DLS joint-limit enforcement.
5. **Custom types survive `gc_reset()`**:
   Call `gc_clear_custom_types(ctx)` only when unloading/rebuilding your archetype registry. Live mechs store a pointer to their registered `GcMechDef`, so `gc_clear_custom_types()` automatically despawns active mechs first.

---

## 5. Game Frame Loop & Mapping Poses to Your Renderer

1. **Pack inputs** (`GC_INPUT_STRIDE = 12` floats per mech, matching spawn order `0..gc_count()-1`):
   ```c
   float* in = &inputs[i * GC_INPUT_STRIDE];
   in[GC_IN_FWD]    = fwd;    // -1..1 (mech-relative)
   in[GC_IN_STRAFE] = strafe; // -1..1
   in[GC_IN_TURN]   = turn;   // -1..1 (+ turns toward right vector / -yaw)
   in[GC_IN_RUN]    = run_held ? 1.0f : 0.0f;
   in[GC_IN_DRIFT]  = slide_held ? 1.0f : 0.0f;
   in[GC_IN_JUMP]   = jump_pressed ? 1.0f : 0.0f;
   in[GC_IN_AIM]    = aim_pitch_rad;
   ```
   *(All input fields are documented in [API_REFERENCE.md](API_REFERENCE.md).)*
2. **Step in parallel**:
   ```c
   gc_step(ctx, dt, inputs, mech_count);
   ```
   `gc_step` internally substeps up to `cfg.max_substep` (default `1/60 s`, clamped frame `dt <= 0.1 s`).
3. **Read poses**:
   - Allocate a pose buffer sized to the sum of `info.floats` (or `gc_floats_per_mech(ctx, i)`).
   - `gc_poses(ctx, out_buf, max_floats)` packs mechs in parallel when `count >= 16`.
   - For each mech's slice `const float* p`:
     - Hull world position: `(p[GC_POSE_POS], p[GC_POSE_POS+1], p[GC_POSE_POS+2])`
     - Orthonormal body basis columns (`X = right`, `Y = up`, `Z = fwd`): notice `gaitcore`'s `right = fwd x up` is left-handed if placed as `+X` with `+Z` forward, so negate `right` (`{-p[3], -p[4], -p[5]}`) as the `+X` column for a standard right-handed rotation matrix (`[ -right | up | fwd ]`, determinant `+1`).
     - Per-leg slice starts at `p + GC_HEADER_FLOATS + leg * (info.joints_per_leg * 3 + GC_LEG_EXTRA_FLOATS)`:
       - `joints[0 .. joints_per_leg-1]` are **world-space 3D joint positions** from hip mount (`0`) down to foot tip (`joints_per_leg - 1`).
       - Align each limb mesh segment along `(joint[s+1] - joint[s])` with the leg's plane normal `L + joints_per_leg * 3` as the hinge axis / up reference!
       - `planted = L[joints_per_leg * 3 + 3] > 0.5f`
       - `sliding = L[joints_per_leg * 3 + 4] > 0.5f`
       - `swing_t = L[joints_per_leg * 3 + 5]` (`0..1` normalized swing progress)
4. **Consume events (`gc_events`)**:
   Drain footstep (`GC_EV_STEP`), hard landing (`GC_EV_LAND`, `GC_EV_IMPACT`), spark (`GC_EV_SPARK`), boost (`GC_EV_BOOST`), skid (`GC_EV_SKID`), and mantle (`GC_EV_MANTLE`) events once per frame to trigger audio, dust/sparks, and decal trails.

---

## 6. Performance, Multithreading & LOD Tuning

- **Thread count**: `gc_create(0)` uses `gc_hardware_threads()` (clamped `1..64`). You can adjust worker count at runtime via `gc_set_threads(ctx, n)` (outside `gc_step`).
- **Crowd LOD via `GcBenchConfig`**:
  - `leg_ik_hz` (`0` = every substep; e.g. `20` = decoupled 20 Hz IK solve, linearly extrapolating/caching across substeps staggered across mechs).
  - `probe_hz` (default `20 Hz`): forward terrain-lookahead raycasts for situation-aware gait selection.
  - `plane_roll` (`1` on, `0` off): disable radial knee obstacle clearance roll if your terrain lacks waist-high obstacles between legs.
  - `profile` (`0` in shipping builds): disable high-resolution internal stage timers in production to avoid extra clock queries.
- **Measured scaling**: On a 24-thread CPU with a fast read-only grid broadphase, 1,000 mixed 4/6/8-legged mechs (6,284 legs total) step at 60 Hz in ~`1.2–2.6 ms` wall clock per frame.
