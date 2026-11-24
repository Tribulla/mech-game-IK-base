# Architecture & Gait Design

The procedural IK and gait core ships as two matching implementations—multithreaded C11 (`IK/c/`) and a pure-Python twin (`IK/python/gaitcore/`)—kept strictly separate from any host renderer or physics world.

---

## 1. Engine vs. Gait Core Split

The core never owns level geometry and never touches GPU/audio state directly. Each frame follows a strict one-way data flow:

```text
  Host Engine / Game (C / C++ / Python)        IK/c/ (or IK/python/gaitcore/)
  ─────────────────────────────────────        ──────────────────────────────
  inputs (12 floats / mech) ─ gc_step() ─────▶ Per-mech parallel update × substeps (<= 1/60s)
                                                 │ Body locomotion (ground | air | crouchjump | mantle)
  read-only world callbacks ◀── queries ──────── │ Orientation & ride-height springs
   (raycast, sphere_push, height)                │ Situation probes & blended gait selector
                                                 │ Rhythm + reactive scheduler, footholds, swing arcs
  meshes / audio / VFX ◀───── gc_poses() ──────┘ Planar leg IK + active-set DLS + knee clearance
                         ◀───── gc_events()
```

- **Why per-mech isolation makes multithreading deterministic**: Each `GcMech` owns its own scratch buffers (`GcWorldQueryCtx`), PRNG state, event vector, and cache-line-aligned (`64 B`) struct so threads never share mutable state or contend on locks. `gc_step()` dispatches `0..count-1` onto a persistent Win32/POSIX work-stealing pool (`gc_thread.c`), then drains events and stats in handle order `0..count-1`, keeping simulation output identical regardless of worker thread count.

### Source Module Mapping

| Responsibility | C11 (`IK/c/src/`) | Python (`IK/python/gaitcore/`) |
|---|---|---|
| Public API & thread pool dispatch | `gc_api.c`, `gc_thread.c` | `api.py` |
| Vector, frame, convex hull, plane fit | `gc_vmath.c` | `vmath.py` |
| World query context & `StubWorld` | `gc_world.c` | `world.py` |
| Planar 2-bone / FABRIK / DLS solver | `gc_ik.c` | `ik.py` |
| Archetype presets & custom descriptors | `gc_mechdefs.c` | `mechdefs.py` |
| Gait phase tables & blending | `gc_gaits.c` | `gaits.py` |
| Situation lookahead & gait ladder | `gc_gaitsel.c` | `gaitsel.py` |
| Quintic swing shaping & leg state | `gc_leg.c` | `leg.py` |
| Foothold scoring, edge/rock avoidance | `gc_foothold.c` | `foothold.py` |
| Knee clearance & radial plane roll | `gc_clearance.c` | `clearance.py` |
| Rhythm + reactive scheduler & support vetoes | `gc_scheduler.c` | `scheduler.py` |
| Hull locomotion, jump/brace, slide, climb | `gc_mech.c` | `mech.py` |

---

## 2. Planar Leg IK (`gc_ik.c` / `ik.py`)

Unconstrained 3D FABRIK twists joints freely around the hip-to-foot line, which makes mechanical linkages look rubbery. Projecting every leg into a single kinematic plane first eliminates twist by construction:
- **Radial (`GC_LEG_PLANE_RADIAL`)**: A coxa joint yaws the plane around the body's up axis toward the foot; femur, tibia, and optional tarsus hinge inside that vertical radial plane. Used by spiders, hexapods, and wide octopods.
- **Sagittal (`GC_LEG_PLANE_SAGITTAL`)**: The plane stays parallel to body forward while a constrained hip roll tilts it sideways. Used by bipeds and under-hull tanks.
- **2-bone & same-bend 3-bone**: Solved analytically via the law of cosines (3-bone splits hinge bend across knee and ankle when `bend_sides` match), avoiding iterative jitter on the common case.
- **General chains**: Warm-started planar FABRIK from the prior frame's pose, preserving exact bone lengths.
- **Joint limits (`_dls`)**: Whenever the unconstrained solve breaks any angle bound in `GcLimits` (or reaches an interior singularity), an active-set damped least-squares solver steps cleanly from the last legal joint configuration. The same routine backs trial foothold feasibility checks.

---

## 3. Rhythm + Reactive Gait Scheduler (`gc_scheduler.c` / `scheduler.py`)

Purely phase-driven central pattern generators break on rough ground; purely distance-triggered steppers look twitchy. The scheduler fuses both:

1. **Gait clock & phase tables**: Each archetype defines `offsets`, `duty`, `stride`, `[cycle_min, cycle_max]`, `min_support`, `adjacent_lock`, `static_stable`, `leap`, and synced `groups`.
2. **Rhythmic requests**: Legs request lift when their swing window opens.
3. **Reactive & predictive triggers**: Legs immediately request a step if they drift past their stance-path zone, pass `97%` reach, or are predicted (`stretch + stretch_rate * swing_time * 0.6`) to over-stretch before a swing could finish.
4. **Rest settling**: When stopped, out-of-place feet step cleanly back to rest; if the rest spot itself sits on a rock edge or void, the leg marks `settled_at` so it never ping-pongs in place.
5. **Lift vetoes (`can_lift`)**: Checks minimum support leg count, neighbor adjacency lock (preventing same-side or paired neighbors from lifting together in alternating gaits), static CoM-vs-support-hull margin (`crawl`/`wave`), and wall-climbing support rules (`>= half` legs down). An over-stretched leg (`> 1.04` reach or stuck at a limit for `> 0.1 s`) drops from support accounting and is allowed to rescue-step while other swinging legs accelerate touchdown.
6. **Futile-step suppression**: Before lifting, the target foothold is queried; steps landing within `0.12 m` of the current planted spot are deferred (`0.25 s` cooldown) to eliminate vibrating steps on clutter.

---

## 4. Situation-Aware Gait Selection (`gc_gaitsel.c` / `gaitsel.py`)

Hard-coding one gait per speed button either looks sloppy on stairs or trips at cliff edges. Instead, each mech climbs or drops along a gait ladder based on speed, ground contact, and forward terrain:
`static (crawl / wave)` -> `walk (walk / ripple / metachronal)` -> `fast (trot_run / tripod_run / tetrapod_run)` -> `gallop (bound)`

- **Speed thresholds + anti-flap hysteresis**: Upshifts wait for a dwell timer (`wait`), while rough-ground downshifts apply immediately and double `relax_wait` on repeated slips so noisy clutter never oscillates between gaits.
- **Route lookahead probes**: At `probe_hz` (default `20 Hz`), three 5-ray clusters probe along the predicted turning arc up to braking distance (`<= 42 m`). Because tilted smooth ramps have consistent local normals across three adjacent samples, they are distinguished from jagged rubble and sheer drops (`void_cnt`), capping both the gait rung and body speed to what that gait can sustain.
- **Smooth gait blending (`GaitBlend`)**: Transitions wrap phase offsets modulo `1.0` over ~1 cycle, interpolating `duty`, `stride`, `leap`, and lift height while tracking live duty in `min_support` until nearly complete so support vetoes don't fight mid-transition steps.

---

## 5. Foothold Selection & Obstacle Avoidance (`gc_foothold.c`, `gc_clearance.c`)

When planning a touchdown:
1. **Predicted hip pose**: Uses velocity, smoothed acceleration, and yaw rate at projected touchdown so high-speed feet land ahead of where the hip will be (MIT Cheetah placement principle).
2. **Line of sight + downward support probe**: Prevents landing through walls; climbers accept wall faces for convex/concave corners.
3. **Edge check + Nudge**: 4 short perimeter probes detect steep steps/edges (`> 0.2 m`). Rejected spots try 8 local nudges (~2 foot widths) first so feet step *onto* rock tops instead of dodging around them, falling back to a two-ring candidate search scored by reach distance, slope, edge safety, and step height.
4. **Clearance**: Checks foot sphere collisions and hip-to-ankle line-of-sight; trial-solves candidates at predicted touchdown to verify knees/ankles remain clear of rocks; radially rolls the leg plane away from nearby lateral rock collisions when enabled (`plane_roll`).

---

## 6. Body Controller & Locomotion Modes (`gc_mech.c` / `mech.py`)

- **Support plane & height spring**: Fits a weighted plane through planted feet and late swing targets in the heading support frame, driving `ride_height` with a critically-damped spring plus belly clearance checks (capsule spheres sized within hull thickness so thin walls cannot catch the belly).
- **Dynamic CoM balance (`GC_POSE_SUPPORT`)**: Checks CoM projection against the convex hull of supporting feet; standing/creeping off an edge gradually releases support and tips cleanly into ballistic flight instead of levitating off the void.
- **Crouch & Jump / Impact Bracing**:
  - `jump` sinks into a leg-verified crouch (`0.2–0.3 s`), launches upward at `jump_speed`, predicts landing (`t`, `point`, `normal`, `speed`) along the ballistic arc, preserves airborne hull pitch/look attitude instead of fighting upright mid-air, reaches feet out into a landing-normal-aligned brace pose just before touchdown, and fires high-authority leg righting + spring absorption the instant feet plant.
- **Quick Slide (`drift`)**: High-speed directional slide where external slide momentum decays with grip while body yaw swings smoothly, emitting skid marks and sparks on skating planted feet plus a directional burst when lining up on release.
- **Mantle, Wire Climb & Wall Climb**: Ledge detection triggers scripted obstacle vaults; taller walls (`<= wire_max`) fire wire anchors to winch over the lip; `climber != 0` mechs pitch onto vertical walls and wrap over convex top edges.
