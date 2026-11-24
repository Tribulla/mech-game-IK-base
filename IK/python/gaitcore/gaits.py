"""Gait pattern library.

Legs are always indexed in pairs from front to back:  L0, R0, L1, R1, L2, R2 ...
`offsets[i]` places leg i in the cycle: its swing starts when the gait clock
reaches (1 - offsets[i]) mod 1 (the scheduler tests (phase + offset) % 1 < swing).
`duty` is the stance fraction (duty > 0.5 = walking, < 0.5 = flight phases).

The scheduler (see mech.py) is a hybrid of a central pattern generator (these
phase tables give the rhythm and the nice readable footfall pattern) and a
reactive layer (feet still step early when they are about to over-stretch, and
lift-off is vetoed when it would break the support rules below).
"""


class Gait:
    __slots__ = ("name", "offsets", "duty", "stride", "cycle_min", "cycle_max",
                 "min_support", "adjacent_lock", "static_stable", "lift", "pair_sync", "reach",
                 "leap", "stride_pitch", "groups", "duty_fast", "phase_lock")

    def __init__(self, name, offsets, duty, stride, cycle_min, cycle_max,
                 min_support=1, adjacent_lock=True, static_stable=False, lift=1.0,
                 pair_sync=False, reach=1.0, leap=1.0, stride_pitch=1.0, groups=None, duty_fast=None,
                 phase_lock=False):
        self.name = name
        self.offsets = list(offsets)
        self.duty = duty
        self.stride = stride            # preferred stride length (m, scaled per mech)
        self.cycle_min = cycle_min      # s
        self.cycle_max = cycle_max      # s
        self.min_support = min_support  # legs that must stay planted when one lifts
        self.adjacent_lock = adjacent_lock  # neighbours may not swing together
        self.static_stable = static_stable  # keep CoM inside support polygon
        self.lift = lift                # step height multiplier
        self.pair_sync = pair_sync      # left/right legs of a pair lift and land together
        self.reach = reach              # how far ahead feet may land (x the step zone)
        self.leap = leap                # flight height, x mechdefs `bound` (min_support 0 gaits)
        self.stride_pitch = stride_pitch  # hull pitch with the stride, x mechdefs `bound_pitch`
        # legs that lift and land together (the bound's left/right pairs, the
        # fast trot's diagonal pairs, alternating tripods / tetrapods)
        if groups is None and pair_sync:
            groups = [[2 * k, 2 * k + 1] for k in range(len(offsets) // 2)]
        self.groups = groups
        # duty at full walking speed: the stance shortens as the gait speeds
        # up (a slow trot always has a pair down, a fast one floats between)
        self.duty_fast = duty_fast
        # every swing lands when the clock starts its stance, however early
        # the foot was released: keeps alternating groups half a cycle apart
        # (released feet would otherwise land early and drift into step)
        self.phase_lock = phase_lock


def biped():
    return {
        "stalk": Gait("stalk", [0.0, 0.5], 0.78, 1.2, 1.2, 1.8, min_support=1, lift=0.7),
        "walk":  Gait("walk",  [0.0, 0.5], 0.62, 2.3, 0.85, 1.25, min_support=1),
        "run":   Gait("run",   [0.0, 0.5], 0.36, 3.8, 0.52, 0.72, min_support=0,
                      adjacent_lock=False, lift=1.3),
    }


def quadruped():
    #            LF    RF    LH    RH
    return {
        "crawl":  Gait("crawl",  [0.75, 0.25, 0.0, 0.5], 0.82, 1.1, 1.3, 2.0,
                       min_support=3, static_stable=True, lift=0.8),
        "walk":   Gait("walk",   [0.75, 0.25, 0.0, 0.5], 0.66, 1.9, 0.9, 1.4, min_support=2),
        "trot":   Gait("trot",   [0.0, 0.5, 0.5, 0.0], 0.50, 2.8, 0.55, 0.85,
                       min_support=2, adjacent_lock=False, lift=1.1),
        # fast trot (high-speed walking): the diagonal pairs (LF+RH,
        # RF+LH) lift and land together; at a brisk walk one pair is always
        # down (duty 0.5), towards full speed the stance shortens (0.27) and
        # the hull floats a moment between pairs, a low hop with a level hull
        "trot_run": Gait("trot_run", [0.0, 0.5, 0.5, 0.0], 0.5, 3.0, 0.34, 0.6,
                         min_support=0, adjacent_lock=False, lift=1.2, reach=1.6,
                         leap=0.06, stride_pitch=0.0, groups=[[0, 3], [1, 2]], duty_fast=0.27,
                         phase_lock=True),
        "gallop": Gait("gallop", [0.5, 0.6, 0.0, 0.1], 0.36, 4.0, 0.42, 0.6,
                       min_support=0, adjacent_lock=False, lift=1.3),
        # bound (gallop): legs move in left/right pairs.  Stance: front pair
        # 0.0-0.4, rear pair 0.25-0.65, then all four are off the ground
        # 0.65-1.0: the rear push-off launches the hull (Mech._update_flight),
        # the front pair lands far ahead, the rear pair a beat later.
        "bound":  Gait("bound",  [0.6, 0.6, 0.35, 0.35], 0.4, 3.4, 0.5, 0.85,
                       min_support=0, adjacent_lock=False, lift=1.6,
                       pair_sync=True, reach=2.2),
    }


def hexapod():
    #           L0     R0     L1     R1     L2   R2
    return {
        "wave":   Gait("wave",   [0.667, 0.167, 0.833, 0.333, 0.0, 0.5], 0.86, 0.9, 1.0, 1.8,
                       min_support=5, static_stable=True, lift=0.8),
        "ripple": Gait("ripple", [0.333, 0.833, 0.667, 0.167, 0.0, 0.5], 0.66, 1.3, 0.6, 1.1,
                       min_support=4),
        "tripod": Gait("tripod", [0.0, 0.5, 0.5, 0.0, 0.0, 0.5], 0.50, 1.8, 0.34, 0.6,
                       min_support=3, adjacent_lock=False, lift=1.1),
        # fast tripod (high-speed walking): alternating tripods lift and
        # land together, duty 0.5 -> 0.3 with speed (a running insect)
        "tripod_run": Gait("tripod_run", [0.0, 0.5, 0.5, 0.0, 0.0, 0.5], 0.5, 2.6, 0.32, 0.55,
                           min_support=0, adjacent_lock=False, lift=1.2, reach=1.6,
                           leap=0.06, stride_pitch=0.0, groups=[[0, 3, 4], [1, 2, 5]], duty_fast=0.3,
                           phase_lock=True),
        # bound (gallop): left/right pairs land front -> middle -> rear and
        # push off in the same order; all six are off the ground 0.65-1.0
        "bound":  Gait("bound",  [0.6, 0.6, 0.48, 0.48, 0.35, 0.35], 0.4, 2.8, 0.45, 0.8,
                       min_support=0, adjacent_lock=False, lift=1.5,
                       pair_sync=True, reach=2.0),
    }


def octopod():
    #              L0     R0     L1    R1    L2     R2     L3   R3
    return {
        "wave":        Gait("wave", [0.625, 0.125, 0.75, 0.25, 0.875, 0.375, 0.0, 0.5], 0.86,
                            1.0, 1.1, 2.0, min_support=6, static_stable=True, lift=0.8),
        "metachronal": Gait("metachronal", [0.25, 0.75, 0.5, 0.0, 0.75, 0.25, 0.0, 0.5], 0.70,
                            1.5, 0.6, 1.2, min_support=5),
        "tetrapod":    Gait("tetrapod", [0.0, 0.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.0], 0.50,
                            2.0, 0.36, 0.65, min_support=4, adjacent_lock=False, lift=1.1),
        # fast tetrapod (high-speed walking): alternating sets of four
        # lift and land together, duty 0.5 -> 0.32 with speed (a running spider)
        "tetrapod_run": Gait("tetrapod_run", [0.0, 0.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.0], 0.5,
                             2.4, 0.4, 0.6, min_support=0, adjacent_lock=False, lift=1.15,
                             reach=1.5, leap=0.1, stride_pitch=0.0,
                             groups=[[0, 3, 4, 7], [1, 2, 5, 6]], duty_fast=0.32, phase_lock=True),
        # bound (heavy gallop): left/right pairs land front -> rear, push off
        # in the same order, all eight off the ground at the end of the cycle
        "bound":       Gait("bound", [0.6, 0.6, 0.52, 0.52, 0.44, 0.44, 0.35, 0.35], 0.4,
                            2.4, 0.45, 0.8, min_support=0, adjacent_lock=False, lift=1.3,
                            pair_sync=True, reach=1.8),
    }


def phase_lerp(a, b, t):
    """Interpolate phases on the unit circle (shortest way)."""
    d = (b - a + 0.5) % 1.0 - 0.5
    return (a + d * t) % 1.0
