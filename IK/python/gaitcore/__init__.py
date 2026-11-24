"""gaitcore - procedural IK + gait system for multi-legged mechs.

Modules
    vmath       tiny vector / frame math
    ik          planar-constrained leg IK (analytic 2-bone, 2D FABRIK, DLS)
    gaits       gait phase tables (biped, quadruped, hexapod, octopod)
    mechdefs    9 general-purpose mech archetypes (pure data)
    mech_const  shared physical constants, event IDs, debug roles, Input state
    gaitsel     forward terrain lookahead, gait hysteresis, and phase blending
    leg         per-leg stance / swing state + swing trajectory
    foothold    foothold probing, rock-top nudges, and edge/wall filters
    clearance   obstacle plane-roll clearance and air landing braces
    scheduler   step scheduling, swing arc tracking, support guards, takeoff
    mech        body sim, slide physics, climbing, orientation, packed pose
    world       raycast / collision interface (native engine or python stub)
    api         flat packed-float API matching gaitcore.h
"""
