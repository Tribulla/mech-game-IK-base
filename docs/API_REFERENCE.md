# API Reference (`gaitcore.h`, `gaitcore` Java, & `gaitcore.api`)

Complete reference for the C11 public API ([../IK/c/include/gaitcore.h](../IK/c/include/gaitcore.h)), the Java 17+ package ([../IK/java/src/main/java/gaitcore/](../IK/java/src/main/java/gaitcore/)), and the Python package ([../IK/python/gaitcore/api.py](../IK/python/gaitcore/api.py)).

---

## 1. Constants & Enums

### Stream & Buffer Strides

| Constant | Value | Meaning |
|---|---|---|
| `GC_INPUT_STRIDE` | `12` | `float`s per mech passed to `gc_step()` |
| `GC_HEADER_FLOATS` | `28` | `float`s in each mech's packed pose header |
| `GC_LEG_EXTRA_FLOATS` | `6` | `plane_normal(3) + planted + sliding + swing_t` after joint positions |
| `GC_EVENT_FLOATS` | `11` | `float`s per packed event in `gc_events()` |
| `GC_DEBUG_LINE_FLOATS` | `9` | `float`s per debug segment in `gc_debug_lines()` |
| `GC_PROFILE_FLOATS` | `61` | `16` header + `9 * 5` per built-in preset breakdown floats |
| `GC_MAX_LEGS_PUBLIC` | `8` | Maximum legs in `GcMechDesc` |
| `GC_MAX_SEGS_PUBLIC` | `4` | Maximum leg segments in `GcMechDesc` |

### Input Channels (`GC_IN_*`)

Per mech `i`, write to `inputs[i * GC_INPUT_STRIDE + channel]`:

| Enum | Index | Range / Meaning |
|---|---|---|
| `GC_IN_FWD` | `0` | Forward/backward throttle (`-1..1`, mech-relative) |
| `GC_IN_STRAFE` | `1` | Lateral strafe (`-1..1`, scaled `0.75` laterally) |
| `GC_IN_TURN` | `2` | Turn rate command (`-1..1`, `+` turns toward body right) |
| `GC_IN_RUN` | `3` | `> 0.5`: sprint / gallop mode |
| `GC_IN_SLOW` | `4` | `> 0.5`: careful / crawl mode |
| `GC_IN_CROUCH` | `5` | Rising edge (`0 -> 1`): toggle crouch |
| `GC_IN_JUMP` | `6` | Rising edge (`0 -> 1`): crouch-jump (jumps off walls while climbing) |
| `GC_IN_DRIFT` | `7` | `> 0.5` while fast: power slide |
| `GC_IN_CLIMB` | `8` | Rising edge (`0 -> 1`): toggle wall-climb mode (if `climber != 0`) |
| `GC_IN_IMPACT` | `9` | Rising edge (`0 -> 1`): apply lateral balance shove impulse |
| `GC_IN_GAIT` | `10` | Rising edge (`0 -> 1`): cycle forced gait override (back to auto after last gait) |
| `GC_IN_AIM` | `11` | Aim elevation pitch in radians (`+ = up`, clamped `±1.53 rad`) |

### Packed Pose Layout (`GC_POSE_*`)

Each mech occupies `info.floats = GC_HEADER_FLOATS + info.legs * (info.joints_per_leg * 3 + GC_LEG_EXTRA_FLOATS)` floats:

#### Header (`0..27`)

| Enum | Offset | Count | Meaning |
|---|---|---|---|
| `GC_POSE_POS` | `0` | `3` | Hull center position in world space `(x, y, z)` |
| `GC_POSE_RIGHT` | `3` | `3` | Body `right` axis (`fwd x up`) |
| `GC_POSE_UP` | `6` | `3` | Body `up` axis |
| `GC_POSE_FWD` | `9` | `3` | Body `fwd` axis |
| `GC_POSE_SPEED` | `12` | `1` | Ground/planar speed (m/s) |
| `GC_POSE_STATE` | `13` | `1` | `GC_STATE_GROUND (0)`, `GC_STATE_AIR (1)`, `GC_STATE_MANTLE (2)`, `GC_STATE_CROUCHJUMP (3)` |
| `GC_POSE_FLAGS` | `14` | `1` | Bitmask of `GC_FLAG_*` |
| `GC_POSE_PLANTED` | `15` | `1` | Number of currently planted feet |
| `GC_POSE_VEL` | `16` | `3` | Linear velocity `(vx, vy, vz)` (m/s) |
| `GC_POSE_PHASE` | `19` | `1` | Gait clock phase `[0, 1)` |
| `GC_POSE_DRIFT` | `20` | `1` | Slide blend amount `[0, 1]` |
| `GC_POSE_GAIT` | `21` | `1` | Active gait index |
| `GC_POSE_SUPPORT` | `22` | `1` | Signed CoM-to-support-hull margin (m, negative = outside) |
| `GC_POSE_CYCLE` | `23` | `1` | Current gait cycle duration (s) |
| `GC_POSE_WIRE` | `24` | `1` | `1.0` when wire-winching up a wall, else `0.0` |
| `GC_POSE_ANCHOR` | `25` | `3` | World-space wire anchor lip point `(x, y, z)` |

#### Per-Leg Record (`l = 0 .. info.legs - 1`)

Starting at `base = GC_HEADER_FLOATS + l * (info.joints_per_leg * 3 + GC_LEG_EXTRA_FLOATS)`:
- `[0 .. joints_per_leg*3 - 1]`: world-space 3D joint positions `[hip_mount, ..., knee, ..., foot_tip]` (`joints_per_leg = 1 + (coxa > 0 ? 1 : 0) + num_segments`)
- `[joints_per_leg*3 + 0 .. +2]`: leg IK plane unit normal `(nx, ny, nz)`
- `[joints_per_leg*3 + 3]`: `planted` (`1.0` planted, `0.0` swing/air)
- `[joints_per_leg*3 + 4]`: `sliding` (`1.0` foot skating, `0.0` static/swing)
- `[joints_per_leg*3 + 5]`: `swing_t` (`0..1` normalized swing phase)

### Pose Flags (`GC_FLAG_*`)

`GC_FLAG_DRIFT (1)`, `GC_FLAG_WALL (2)`, `GC_FLAG_CROUCH (4)`, `GC_FLAG_CLIMB (8)`, `GC_FLAG_SUPPORTED (16)`, `GC_FLAG_SLOW (32)`, `GC_FLAG_RUN (64)`, `GC_FLAG_WIRE (128)`, `GC_FLAG_LEAP (256)`

### Events (`GC_EV_*`) & Debug Roles (`GC_DBG_*`)

Each event is `11` floats: `[type, px, py, pz, nx, ny, nz, strength, ex, ey, ez]`:
- `GC_EV_STEP (1)`: foot plant impact
- `GC_EV_SPARK (2)`: foot slide sparks (`(ex,ey,ez)` = foot velocity)
- `GC_EV_LAND (3)`: landing after flight (`strength` = impact speed fraction)
- `GC_EV_BOOST (4)`: slide release boost
- `GC_EV_IMPACT (5)`: hard landing / shove impulse
- `GC_EV_SKID (6)`: skid mark segment from `(px,py,pz)` to `(ex,ey,ez)` with width `strength`
- `GC_EV_MANTLE (7)`: ledge mantle triggered

Each debug line is `9` floats: `[ax, ay, az, bx, by, bz, role, 0, 0]` with `role` in `GC_DBG_LINE (0)` through `GC_DBG_AXIS_Z (12)`.

---

## 2. Public C Functions (`gaitcore.h`)

### Lifecycle & World Binding
- `int gc_hardware_threads( void )` — hardware concurrency detected (clamped `1..64`).
- `GcContext* gc_create( int num_threads )` — creates context and persistent thread pool (`<= 0` = auto).
- `void gc_destroy( GcContext* ctx )` — frees mechs, custom types, buffers, and thread pool.
- `void gc_reset( GcContext* ctx )` — despawns all active mechs and clears transient stats (retains registered custom mech types).
- `void gc_set_world( GcContext* ctx, const GcWorldCallbacks* world )` — installs read-only world query callbacks.
- `void gc_set_threads( GcContext* ctx, int num_threads )` / `int gc_get_threads( const GcContext* ctx )` — resizes/queries worker pool (call between `gc_step` calls).

### Built-in Presets & Custom Mech Archetypes
- `int gc_kind_count( void )` — number of built-in presets (`9`).
- `const char* gc_kind_name( int kind )` — preset name (`"LYNX"`, `"STRIKER"`, `"STRIDER"`, `"GECKO"`, etc.).
- `void gc_mech_desc_init_default( GcMechDesc* out_desc, int num_legs )` — fills valid defaults for `2`, `4`, `6`, or `8` legs.
- `int gc_mech_desc_from_preset( int preset_kind, GcMechDesc* out_desc )` — copies a built-in preset (`0..8`) into an editable descriptor; returns `1` on success, `0` if invalid.
- `int gc_register_mech_type( GcContext* ctx, const GcMechDesc* desc )` — validates & registers custom type, returning kind ID `>= gc_kind_count()`, or `-1` if invalid.
- `int gc_context_kind_count( const GcContext* ctx )` / `const char* gc_context_kind_name( const GcContext* ctx, int kind )` — queries built-in + custom kinds registered in `ctx`.
- `void gc_clear_custom_types( GcContext* ctx )` — despawns active mechs and frees all custom types registered in `ctx`.

### Spawning & Introspection
- `int gc_spawn( GcContext* ctx, int kind, float x, float y, float z, float yaw )` — spawns at ground surface `(x,y,z)`, returns handle `0..N-1` or `-1` if invalid.
- `int gc_spawn_custom( GcContext* ctx, const GcMechDesc* desc, float x, float y, float z, float yaw )` — registers (or reuses an identical registered custom def) and spawns one instance.
- `int gc_respawn( GcContext* ctx, int handle, int kind, float x, float y, float z, float yaw )` — replaces mech `handle` in-place; returns `handle` or `-1`.
- `int gc_despawn_to( GcContext* ctx, int target_count )` — trims tail mechs down to `target_count`.
- `int gc_count( const GcContext* ctx )` — active mech count.
- `int gc_info( const GcContext* ctx, int handle, GcMechInfo* out )` — returns static dimensions, leg count, widths, speeds, and `floats`.
- `int gc_floats_per_mech( const GcContext* ctx, int handle )` — pose float stride for `handle`.

### Simulation, Extraction & Profiling
- `void gc_step( GcContext* ctx, float dt, const float* inputs, int mech_count )` — advances all active mechs in parallel (`dt` clamped to `<= 0.1 s`, substepped by `cfg.max_substep`).
- `int gc_poses( GcContext* ctx, float* out_buf, int max_floats )` — packs poses sequentially into `out_buf` (parallelized when `count >= 16`); returns floats written.
- `int gc_events( GcContext* ctx, float* out_buf, int max_floats )` — copies and clears accumulated frame events; returns floats written.
- `int gc_debug_lines( const GcContext* ctx, int handle, float* out_buf, int max_floats )` — writes debug lines for `handle` plus recorded rays when `gc_set_debug(ctx, 1)` is active.
- `void gc_set_debug( GcContext* ctx, int on )` — enables/disables debug ray recording.
- `int gc_stats( const GcContext* ctx, int handle, GcMechStats* out )` — mode, gait label (including active blend & situation reason), state, duty, cycle, max stretch, rays.
- `void gc_set_bench_config( GcContext* ctx, const GcBenchConfig* cfg )` — sets `max_substep`, `leg_ik_hz` (`0` = coupled), `probe_hz`, `speed_scale`, `plane_roll`, `profile`.
- `int gc_profile_data( const GcContext* ctx, float* out_buf, int max_floats )` — fills `[0]=step_wall_ms, [1..8]=stages_ms, [9]=substeps, [10]=ik_solves, [11]=ik_cached, [12]=rays, [13]=poses_ms, [14]=legs, [15]=mechs, [16..60]=9x5 per-preset stats`.
- `void gc_reset_query_stats( GcContext* ctx )` / `GcQueryStats gc_get_query_stats( const GcContext* ctx )` / `float gc_last_cpu_ms( const GcContext* ctx )`.

### Built-In Stub World (`GcStubWorld`, Tests & Headless Tools)
- `gc_stub_world_create(double ground_y, int has_ground)`, `gc_stub_world_destroy`, `gc_stub_world_add_box`, `gc_stub_world_add_obox`, `gc_stub_world_callbacks`.

---

## 3. Built-In Preset Kinds (`0..8`)

| Kind ID | Name | Legs | Segments | IK Plane | Highlights |
|---|---|---|---|---|---|
| `0` | `LYNX` | 4 | 2 + coxa | Radial | Light recon quadruped (`35.7 m/s` run), `9 m` wire climb anchors |
| `1` | `STRIKER` | 4 | 3 + coxa | Radial | High-speed assault quadruped (`47.6 m/s` run), high jump (`17.9 m/s`), `10 m` wire climb |
| `2` | `STRIDER` | 2 | 2 | Sagittal | Combat biped (`13.5 m/s` run) with hip roll & twin shoulder cannons |
| `3` | `GECKO` | 4 | 2 + coxa | Radial | Surface-adhesion spider scout (`climber=1`, full wall/ceiling walking) |
| `4` | `BASTION` | 8 | 2 | Sagittal | Heavy tank under-hull octopod (`mass 4.0`) |
| `5` | `WARDEN` | 8 | 3 + coxa | Radial | Wide artillery octopod, balanced multi-segment stance |
| `6` | `MANTIS` | 6 | 3 + coxa | Radial | High-speed hunter hexapod (`47.6 m/s` run) |
| `7` | `SCARAB` | 6 | 2 + coxa | Radial | Compact, agile utility scout hexapod |
| `8` | `COLOSSUS` | 8 | 3 + coxa | Radial | Super-heavy siege octopod (`size 1.6`, `mass 6.0`, slow cadence) |

---

## 4. Python Package (`IK/python/gaitcore`)

- `gaitcore.api`: `init()`, `reset(reload_modules=False)`, `kinds()`, `spawn(kind, x, y, z, yaw)`, `respawn(h, kind, x, y, z, yaw)`, `despawn_to(n)`, `info(h)`, `floats_per_mech(h)`, `step(dt, inputs_bytes)`, `poses() -> bytes`, `events() -> bytes`, `debug_lines(h) -> bytes`, `set_debug(flag)`, `stats(h)`, `set_bench_config(...)`, `profile_data() -> bytes`, `hot_reload()`.
- Standalone Python verification tools under `IK/python/tools/`:
  - `headless_sim.py` — scripted flat/stairs/ledge/wall/drift/jump/settle regression suite.
  - `limit_check.py` — joint-limit margin and worst planted-foot error audit.
  - `fuzz_check.py` — randomized obstacle driving for stuck legs and rock clipping (`--world rubble|boulders`).
  - `foothold_check.py` — verifies feet plant on top of rubble/boulders rather than weaving around.
  - `glitch_check.py` — detects idle stepping, quick re-steps, hanging feet, and gallop pose spikes.
  - `gait_select_check.py` — audits situation-driven gait transitions without flapping.
  - `gait_film.py` — renders frame-by-frame side/top PNG strips + footfall strips.

---

## 5. Java 17+ Package (`IK/java/src/main/java/gaitcore`)

All Java public types mirror the C API strides, pose offsets, input channels, and flags bit-for-bit using camelCase methods:

- `GaitCore`: constants (`INPUT_STRIDE`, `HEADER_FLOATS`, `LEG_EXTRA_FLOATS`, `EVENT_FLOATS`, `DEBUG_LINE_FLOATS`, `PROFILE_FLOATS`, `IN_*`, `POSE_*`, `FLAG_*`, `STATE_*`, `EV_*`, `DBG_*`, `LEG_PLANE_*`) plus `hardwareThreads()`, `kindCount()`, `kindName(int)`.
- `GcContext` (`AutoCloseable`): `GcContext.create(int numThreads)`, `close()`, `reset()`, `setWorld(GcWorldCallbacks)`, `setThreads(int)`, `getThreads()`, `registerMechType(GcMechDesc)`, `contextKindCount()`, `contextKindName(int)`, `clearCustomTypes()`, `spawn(...)`, `spawnCustom(...)`, `respawn(...)`, `despawnTo(int)`, `count()`, `floatsPerMech(int)`, `info(int, GcMechInfo)`, `setBenchConfig(GcBenchConfig)`, `step(float, float[], int)`, `poses(float[], int)`, `events(float[], int)`, `debugLines(int, float[], int)`, `setDebug(boolean)`, `stats(int, GcMechStats)`, `profileData(float[], int)`, `resetQueryStats()`, `getQueryStats()`, `lastCpuMs()`.
- `GcWorldCallbacks` & `GcStubWorld`: thread-safe read-only `raycast(...)`, `spherePush(...)`, and optional `terrainHeight(...)`; `GcStubWorld` supports flat ground, AABBs (`addBox`), and rotated OBBs (`addOBox`).
- `GcMechDesc`: mutable custom mech descriptor matching `GcMechDesc` in `gaitcore.h`, with `createDefault(int numLegs)`, `fromPreset(int, GcMechDesc)`, `copy()`, `copyFrom(...)`, `equals`/`hashCode`.
- `GcBenchConfig`, `GcMechInfo`, `GcMechStats`, `GcQueryStats`: public configuration and telemetry data classes.
