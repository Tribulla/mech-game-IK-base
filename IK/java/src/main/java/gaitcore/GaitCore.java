package gaitcore;

/**
 * Public API constants, stream strides, enum codes, and shared simulation timing constants.
 * Kept aligned with {@code IK/c/include/gaitcore.h} and {@code IK/c/src/gc_internal.h}.
 */
public final class GaitCore {

    private GaitCore() {
    }

    // ------------------------------------------------------------------ Public stream strides

    public static final int INPUT_STRIDE = 12;
    public static final int HEADER_FLOATS = 28;
    public static final int LEG_EXTRA_FLOATS = 6;
    public static final int EVENT_FLOATS = 11;
    public static final int DEBUG_LINE_FLOATS = 9;
    public static final int PROFILE_HEADER = 16;
    public static final int PROFILE_KINDS = 9;
    public static final int PROFILE_KIND_STRIDE = 5;
    public static final int PROFILE_FLOATS = PROFILE_HEADER + PROFILE_KINDS * PROFILE_KIND_STRIDE;

    public static final int MAX_LEGS_PUBLIC = 8;
    public static final int MAX_SEGS_PUBLIC = 4;

    // ------------------------------------------------------------------ Planar IK modes

    /** Coxa yaws around hull up toward target (splayed spider/hexapod legs). */
    public static final int LEG_PLANE_RADIAL = 0;
    /** Leg plane aligns with hull forward with limited hip roll (bipeds, narrow tanks). */
    public static final int LEG_PLANE_SAGITTAL = 1;

    // ------------------------------------------------------------------ Input channels

    public static final int IN_FWD = 0;
    public static final int IN_STRAFE = 1;
    public static final int IN_TURN = 2;
    public static final int IN_RUN = 3;
    public static final int IN_SLOW = 4;
    public static final int IN_CROUCH = 5;
    public static final int IN_JUMP = 6;
    public static final int IN_DRIFT = 7;
    public static final int IN_CLIMB = 8;
    public static final int IN_IMPACT = 9;
    public static final int IN_GAIT = 10;
    public static final int IN_AIM = 11;

    // ------------------------------------------------------------------ Packed pose header offsets

    public static final int POSE_POS = 0;
    public static final int POSE_RIGHT = 3;
    public static final int POSE_UP = 6;
    public static final int POSE_FWD = 9;
    public static final int POSE_SPEED = 12;
    public static final int POSE_STATE = 13;
    public static final int POSE_FLAGS = 14;
    public static final int POSE_PLANTED = 15;
    public static final int POSE_VEL = 16;
    public static final int POSE_PHASE = 19;
    public static final int POSE_DRIFT = 20;
    public static final int POSE_GAIT = 21;
    public static final int POSE_SUPPORT = 22;
    public static final int POSE_CYCLE = 23;
    public static final int POSE_WIRE = 24;
    public static final int POSE_ANCHOR = 25;

    // ------------------------------------------------------------------ Pose flags

    public static final int FLAG_DRIFT = 1;
    public static final int FLAG_WALL = 2;
    public static final int FLAG_CROUCH = 4;
    public static final int FLAG_CLIMB = 8;
    public static final int FLAG_SUPPORTED = 16;
    public static final int FLAG_SLOW = 32;
    public static final int FLAG_RUN = 64;
    public static final int FLAG_WIRE = 128;
    public static final int FLAG_LEAP = 256;

    // ------------------------------------------------------------------ Locomotion states

    public static final int STATE_GROUND = 0;
    public static final int STATE_AIR = 1;
    public static final int STATE_MANTLE = 2;
    public static final int STATE_CROUCHJUMP = 3;

    // ------------------------------------------------------------------ Event IDs

    public static final int EV_STEP = 1;
    public static final int EV_SPARK = 2;
    public static final int EV_LAND = 3;
    public static final int EV_BOOST = 4;
    public static final int EV_IMPACT = 5;
    public static final int EV_SKID = 6;
    public static final int EV_MANTLE = 7;

    // ------------------------------------------------------------------ Debug line roles

    public static final int DBG_LINE = 0;
    public static final int DBG_LINE_DIM = 1;
    public static final int DBG_INFO = 2;
    public static final int DBG_HIGHLIGHT = 3;
    public static final int DBG_GOOD = 4;
    public static final int DBG_WARNING = 5;
    public static final int DBG_ERROR = 6;
    public static final int DBG_FORCE = 7;
    public static final int DBG_NORMAL = 8;
    public static final int DBG_VELOCITY = 9;
    public static final int DBG_AXIS_X = 10;
    public static final int DBG_AXIS_Y = 11;
    public static final int DBG_AXIS_Z = 12;

    // ------------------------------------------------------------------ Internal math & tuning constants

    static final double PI = Math.PI;
    static final double TAU = Math.PI * 2.0;
    static final double DEG2RAD = PI / 180.0;
    static final double RAD2DEG = 180.0 / PI;

    static final int MAX_LEGS = 8;
    static final int MAX_SEGS = 4;
    static final int MAX_JOINTS = 6;
    static final int MAX_GAITS = 8;
    static final int MAX_GROUPS = 4;
    static final int MAX_GROUP_SIZE = 4;
    static final int MAX_COL_OFFS = 8;
    static final int MAX_HULL_PTS = 16;

    // Heavier-than-earth gravity keeps multi-ton walkers crisp during jumps and bounds.
    static final double G = 20.0;
    static final double A_LAT = 135.0;
    static final double AIM_MAX = 1.53;
    static final double TAKEOFF_RAMP_REACH = 0.36;
    static final double LANDING_RAMP_REACH = 0.68;
    static final double MAX_SUBSTEP_DEFAULT = 1.0 / 60.0;
    static final double MAX_FRAME_DT = 0.1;

    static final double H_GALLOP = 0.15;
    static final double H_FAST = 0.40;
    static final double H_WALK = 0.70;
    static final double SLOPE_FAST = 20.0;
    static final double SLOPE_WALK = 38.0;
    static final double RELAX_T = 1.15;
    static final double RELAX_T_MAX = 1.8;

    static final int PLANE_RADIAL = 0;
    static final int PLANE_SAGITTAL = 1;

    static final int MODE_SLOW = 0;
    static final int MODE_WALK = 1;
    static final int MODE_RUN = 2;

    /** Detected hardware thread count clamped to [1, 64]. */
    public static int hardwareThreads() {
        int n = Runtime.getRuntime().availableProcessors();
        if (n < 1) {
            return 1;
        }
        return Math.min(n, 64);
    }

    /** Number of built-in mech presets (0 .. kindCount() - 1). */
    public static int kindCount() {
        return MechDef.BUILTIN_COUNT;
    }

    /** Name of built-in preset {@code kind}, or empty string if out of range. */
    public static String kindName(int kind) {
        if (kind < 0 || kind >= MechDef.BUILTIN_COUNT) {
            return "";
        }
        return MechDef.BUILTINS[kind].name;
    }

    static double timeSec() {
        return System.nanoTime() * 1e-9;
    }

    static double clamp(double x, double lo, double hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    static double smoothstep(double t) {
        double u = clamp(t, 0.0, 1.0);
        return u * u * (3.0 - 2.0 * u);
    }

    static double damp(double rate, double dt) {
        return 1.0 - Math.exp(-rate * dt);
    }

    static double mod1(double x) {
        double r = x % 1.0;
        if (r < 0.0) {
            r += 1.0;
        }
        if (r >= 1.0) {
            r = 0.0;
        }
        return r;
    }

    static double wrapPi(double a) {
        double r = (a + PI) % TAU;
        if (r < 0.0) {
            r += TAU;
        }
        return r - PI;
    }
}
