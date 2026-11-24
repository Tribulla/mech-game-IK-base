"""Mech archetypes.  Everything here is data: tweak numbers, press F5 in-game.

The built-in roster showcases 2-, 4-, 6-, and 8-legged general-purpose mech
classes across radial and sagittal leg configurations, 2- and 3-segment leg
chains, high-speed bounding, wire anchors, and surface wall-climbing.

gait_by_mode and walk_fast form the gait ladder of gaitsel.GaitSelector:
slow (static) -> walk -> walk_fast -> run (gallop); speed picks the rung and
the ground caps it.

Body-local coordinates: x = right, y = up, z = forward.  Units: metres, seconds.
Legs are listed front-to-back in left/right pairs (L0, R0, L1, R1, ...).

Keys:
  plane       "radial" (coxa yaws the leg plane: spider-like legs splayed out)
              or "sagittal" (plane holds the body's forward axis: legs under the hull)
  coxa        length of the yawing hip segment (radial only)
  lengths     segment lengths inside the leg plane, hip to foot
  sides       bend side per interior joint (+1 knee up/forward, -1 down/back)
  widths      drawing thickness per segment
  ride        body height above the fitted foot plane
  mantle_max  tallest ledge it pulls itself onto with its legs
  wire_max    tallest wall it scales with wire anchors (0 = none)
  climber     walks up walls and ceilings on its feet
  accel       acceleration walking slowly (m/s^2); accel_run while running, and
              while walking faster than the walking gait carries
  walk_fast   gait for walking faster than the walking gait's legs carry (a
              fast trot / tripod / tetrapod: synced leg groups landing in turn,
              brief floats between them at full speed, level hull)
  mass        heaviness (landing dip, effects)
  jump_speed  take-off speed (m/s); apex height = jump_speed**2 / 40
  look_tilt   how far the hull pitches to look up / down relative to the ground (deg, default 25)
  bound       gallop flight: how high the hull leaps (m) at full run in gaits
              with a flight phase (min_support 0), times the gait's `leap`
              (bound 1.0, the fast-walk gaits 0.06); 0 = the hull stays on its legs
  bound_pitch hull pitch (deg) over a gallop stride
  size        gait timing scale (dynamic similarity: big machines step slower)
  faction     0 = Vanguard, 1 = Syndicate (demo team color tag)
  limits      joint limits in degrees (ik.LegIK):
                coxa  max coxa yaw either way from the leg's rest direction
                roll  max sideways hip roll of the leg plane (sagittal legs)
                hip   (min, max) pitch of the first segment.  radial: 0 =
                      horizontal outward, +90 = up; sagittal: 0 = straight
                      down, + = forward
                bend  (min, max) bend of each knee away from straight, + =
                      towards its `sides` direction; one pair for all knees
                      or a list per knee.  min > 0 stops a knee locking
                      straight or flipping; max < 180 stops it folding flat.
              A leg that can't reach its foot within the limits counts as
              over-stretched, and footholds outside them are not chosen.
"""
from . import gaits

VANGUARD, SYNDICATE = 0, 1


def _pairs(rows):
    """rows: list of (mount_xyz, rest_xz, sides_or_None) for the LEFT leg.
    Mirrors each into a right leg.  x given for the left side must be negative."""
    legs = []
    for mount, rest, sides in rows:
        mx, my, mz = mount
        rx, rz = rest
        legs.append(dict(mount=(mx, my, mz), rest=(rx, rz), sides=sides, side=-1))
        legs.append(dict(mount=(-mx, my, mz), rest=(-rx, rz), sides=sides, side=+1))
    return legs


# --- Vanguard ---------------------------------------------------------------

LYNX = dict(
    name="LYNX", kind_id=0, faction=VANGUARD,
    blurb="Light recon quadruped - agile spider legs, wire anchors",
    plane="radial", coxa=0.25,
    lengths=(1.4, 1.6), sides=(1,), widths=(0.22, 0.16),
    limits=dict(coxa=75.0, hip=(-30.0, 75.0), bend=(8.0, 150.0)),
    legs=_pairs([((-0.75, 0.0, 0.95), (-2.0, 2.1), None),
                 ((-0.75, 0.0, -0.95), (-2.0, -2.0), None)]),
    body_half=(0.8, 0.45, 1.25), ride=1.15,
    speeds=dict(slow=10.5, walk=27.3, run=35.7), accel=8.0, accel_run=24.0, turn_rate=3.4,
    gaits=gaits.quadruped(), gait_by_mode=dict(slow="crawl", walk="walk", run="bound"),
    walk_fast="trot_run",
    sprint=None, drift_gait="trot",
    step_height=0.45, step_max=0.7, mantle_max=1.6, wire_max=9.0, climber=False,
    jump_speed=14.1, max_tilt=20.0, look_tilt=25.0, mass=1.0, biped=False, bound=0.45, bound_pitch=8.0,
    zone=1.0, swing_min=0.14, swing_max=0.42,
)

STRIKER = dict(
    name="STRIKER", kind_id=1, faction=VANGUARD,
    blurb="Assault quadruped - 3-seg legs, high jump, wire anchors",
    plane="radial", coxa=0.3,
    lengths=(1.6, 1.5, 0.55), sides=(1, 1), widths=(0.26, 0.2, 0.18),
    limits=dict(coxa=75.0, hip=(-35.0, 70.0), bend=[(5.0, 145.0), (15.0, 150.0)]),
    legs=_pairs([((-0.85, 0.05, 1.1), (-2.3, 2.5), None),
                 ((-0.85, 0.05, -1.1), (-2.3, -2.3), None)]),
    body_half=(0.9, 0.5, 1.45), ride=1.45,
    speeds=dict(slow=14.0, walk=36.4, run=47.6), accel=12.0, accel_run=36.0, turn_rate=4.2,
    gaits=gaits.quadruped(), gait_by_mode=dict(slow="crawl", walk="walk", run="bound"),
    walk_fast="trot_run",
    sprint=None, drift_gait="trot",
    step_height=0.55, step_max=0.8, mantle_max=2.2, wire_max=10.0, climber=False,
    jump_speed=17.9, max_tilt=22.0, look_tilt=25.0, mass=1.0, biped=False, bound=0.75, bound_pitch=10.0,
    zone=1.15, swing_min=0.12, swing_max=0.4,
)

STRIDER = dict(
    name="STRIDER", kind_id=2, faction=VANGUARD,
    blurb="Combat biped - sagittal 2-leg strider with shoulder cannons",
    plane="sagittal", coxa=0.0,
    lengths=(1.35, 1.35), sides=(1,), widths=(0.26, 0.22),
    limits=dict(roll=30.0, hip=(-75.0, 105.0), bend=(8.0, 150.0)),
    legs=_pairs([((-0.48, -0.25, 0.0), (-0.48, 0.0), None)]),
    body_half=(0.55, 0.55, 0.55), ride=1.95,
    speeds=dict(slow=3.5, walk=6.5, run=13.5), accel=9.0, accel_run=20.0, turn_rate=3.6,
    gaits=gaits.biped(), gait_by_mode=dict(slow="stalk", walk="walk", run="run"),
    walk_fast=None,
    sprint=None, drift_gait="walk",
    step_height=0.45, step_max=0.75, mantle_max=1.6, wire_max=0.0, climber=False,
    jump_speed=12.5, max_tilt=16.0, look_tilt=20.0, mass=1.0, biped=True, bound=0.22, bound_pitch=3.0,
    zone=0.9, swing_min=0.12, swing_max=0.36,
)

GECKO = dict(
    name="GECKO", kind_id=3, faction=VANGUARD,
    blurb="Wall-climbing spider scout - surface-adhesion quadruped",
    plane="radial", coxa=0.22,
    lengths=(1.35, 1.50), sides=(1,), widths=(0.20, 0.14),
    limits=dict(coxa=75.0, hip=(-35.0, 80.0), bend=(8.0, 155.0)),
    legs=_pairs([((-0.68, -0.05, 0.85), (-1.85, 1.85), None),
                 ((-0.68, -0.05, -0.85), (-1.85, -1.80), None)]),
    body_half=(0.72, 0.36, 1.05), ride=1.00,
    speeds=dict(slow=10.0, walk=24.0, run=32.0), accel=10.0, accel_run=28.0, turn_rate=4.2,
    gaits=gaits.quadruped(), gait_by_mode=dict(slow="crawl", walk="walk", run="bound"),
    walk_fast="trot_run",
    sprint=None, drift_gait="trot",
    step_height=0.45, step_max=0.75, mantle_max=1.6, wire_max=0.0, climber=True,
    jump_speed=14.5, max_tilt=28.0, look_tilt=25.0, mass=0.85, biped=False, bound=0.40, bound_pitch=7.0,
    zone=0.95, swing_min=0.12, swing_max=0.38,
)

# --- Syndicate --------------------------------------------------------------

BASTION = dict(
    name="BASTION", kind_id=4, faction=SYNDICATE,
    blurb="Armored sagittal octopod - heavy tank with under-hull legs",
    plane="sagittal", coxa=0.0,
    lengths=(1.0, 1.1), sides=(-1,), widths=(0.34, 0.28),
    limits=dict(roll=45.0, hip=(-85.0, 30.0), bend=(10.0, 150.0)),
    legs=_pairs([((-1.35, -0.45, 2.0), (-1.75, 2.15), None),
                 ((-1.35, -0.45, 0.7), (-1.75, 0.75), None),
                 ((-1.35, -0.45, -0.7), (-1.75, -0.75), None),
                 ((-1.35, -0.45, -2.0), (-1.75, -2.05), None)]),
    body_half=(1.45, 0.7, 2.6), ride=1.75,
    speeds=dict(slow=9.8, walk=25.5, run=33.3), accel=3.0, accel_run=9.0, turn_rate=1.8,
    gaits=gaits.octopod(), gait_by_mode=dict(slow="wave", walk="metachronal", run="bound"),
    walk_fast="tetrapod_run",
    sprint=None, drift_gait="tetrapod",
    step_height=0.4, step_max=0.65, mantle_max=1.4, wire_max=0.0, climber=False,
    jump_speed=10.0, max_tilt=14.0, look_tilt=25.0, mass=4.0, biped=False, bound=0.30, bound_pitch=4.0,
    zone=0.8, swing_min=0.16, swing_max=0.45,
)

WARDEN = dict(
    name="WARDEN", kind_id=5, faction=SYNDICATE,
    blurb="Artillery octopod - 3-segment radial legs, heavy turret",
    plane="radial", coxa=0.3,
    lengths=(1.8, 1.7, 0.6), sides=(1, 1), widths=(0.3, 0.24, 0.16),
    limits=dict(coxa=70.0, hip=(-20.0, 65.0), bend=[(35.0, 150.0), (15.0, 145.0)]),
    legs=_pairs([((-1.2, 0.0, 1.5), (-2.7, 2.6), None),
                 ((-1.2, 0.0, 0.5), (-3.0, 0.9), None),
                 ((-1.2, 0.0, -0.5), (-3.0, -0.9), None),
                 ((-1.2, 0.0, -1.5), (-2.7, -2.5), None)]),
    body_half=(1.3, 0.65, 2.2), ride=1.4,
    speeds=dict(slow=9.8, walk=25.5, run=33.3), accel=4.5, accel_run=13.5, turn_rate=2.0,
    gaits=gaits.octopod(), gait_by_mode=dict(slow="wave", walk="metachronal", run="bound"),
    walk_fast="tetrapod_run",
    sprint=None, drift_gait="tetrapod",
    step_height=0.45, step_max=0.75, mantle_max=1.8, wire_max=0.0, climber=False,
    jump_speed=11.0, max_tilt=18.0, look_tilt=25.0, mass=4.0, biped=False, bound=0.35, bound_pitch=5.0,
    zone=0.95, swing_min=0.14, swing_max=0.45,
)

MANTIS = dict(
    name="MANTIS", kind_id=6, faction=SYNDICATE,
    blurb="Hunter hexapod - agile 3-seg flanker with foreleg blades",
    plane="radial", coxa=0.2,
    lengths=(1.0, 1.8, 0.7), sides=(1, 1), widths=(0.18, 0.2, 0.12),
    limits=dict(coxa=75.0, hip=(-25.0, 100.0), bend=[(10.0, 160.0), (10.0, 155.0)]),
    legs=_pairs([((-0.55, -0.35, 0.8), (-1.9, 2.0), None),
                 ((-0.55, -0.35, 0.0), (-2.1, 0.1), None),
                 ((-0.55, -0.35, -0.8), (-1.9, -1.8), None)]),
    body_half=(0.6, 0.45, 1.5), ride=1.45,
    speeds=dict(slow=14.0, walk=36.4, run=47.6), accel=12.0, accel_run=36.0, turn_rate=4.4,
    gaits=gaits.hexapod(), gait_by_mode=dict(slow="wave", walk="ripple", run="bound"),
    walk_fast="tripod_run",
    sprint=None, drift_gait="tripod",
    step_height=0.5, step_max=0.75, mantle_max=1.8, wire_max=0.0, climber=False,
    jump_speed=16.7, max_tilt=24.0, look_tilt=25.0, mass=1.0, biped=False, bound=0.55, bound_pitch=9.0,
    zone=1.0, swing_min=0.11, swing_max=0.38,
)

SCARAB = dict(
    name="SCARAB", kind_id=7, faction=SYNDICATE,
    blurb="Utility scout hexapod - compact frame, sensor head",
    plane="radial", coxa=0.2,
    lengths=(1.2, 1.45), sides=(1,), widths=(0.17, 0.12),
    limits=dict(coxa=75.0, hip=(-25.0, 75.0), bend=(40.0, 165.0)),
    legs=_pairs([((-0.5, -0.1, 0.6), (-1.55, 1.5), None),
                 ((-0.5, -0.1, 0.0), (-1.75, 0.0), None),
                 ((-0.5, -0.1, -0.6), (-1.55, -1.4), None)]),
    body_half=(0.55, 0.38, 0.95), ride=0.95,
    speeds=dict(slow=11.2, walk=29.1, run=38.1), accel=10.0, accel_run=30.0, turn_rate=4.0,
    gaits=gaits.hexapod(), gait_by_mode=dict(slow="wave", walk="ripple", run="bound"),
    walk_fast="tripod_run",
    sprint=None, drift_gait="tripod",
    step_height=0.36, step_max=0.6, mantle_max=1.2, wire_max=0.0, climber=False,
    jump_speed=13.4, max_tilt=26.0, look_tilt=25.0, mass=0.9, biped=False, bound=0.30, bound_pitch=7.0,
    zone=0.75, swing_min=0.11, swing_max=0.38,
)

COLOSSUS = dict(
    name="COLOSSUS", kind_id=8, faction=SYNDICATE,
    blurb="Siege octopod - super-heavy walker with dual cannons",
    plane="radial", coxa=0.45,
    lengths=(2.6, 2.6, 0.9), sides=(1, 1), widths=(0.45, 0.36, 0.24),
    limits=dict(coxa=65.0, hip=(-15.0, 55.0), bend=[(40.0, 150.0), (15.0, 145.0)]),
    legs=_pairs([((-1.85, -0.1, 2.3), (-4.0, 3.7), None),
                 ((-1.85, -0.1, 0.8), (-4.4, 1.3), None),
                 ((-1.85, -0.1, -0.8), (-4.4, -1.3), None),
                 ((-1.85, -0.1, -2.3), (-4.0, -3.5), None)]),
    body_half=(2.0, 1.0, 3.2), ride=2.3,
    speeds=dict(slow=7.0, walk=18.2, run=23.8), accel=2.5, accel_run=7.5, turn_rate=1.2,
    gaits=gaits.octopod(), gait_by_mode=dict(slow="wave", walk="metachronal", run="bound"),
    walk_fast="tetrapod_run",
    sprint=None, drift_gait="tetrapod",
    step_height=0.7, step_max=1.2, mantle_max=2.6, wire_max=0.0, climber=False,
    jump_speed=8.9, max_tilt=12.0, look_tilt=25.0, mass=6.0, biped=False, bound=0.30, bound_pitch=4.0,
    zone=1.5, swing_min=0.22, swing_max=0.65, size=1.6,
)

ALL = [LYNX, STRIKER, STRIDER, GECKO, BASTION, WARDEN, MANTIS, SCARAB, COLOSSUS]
BY_NAME = {d["name"]: d for d in ALL}
