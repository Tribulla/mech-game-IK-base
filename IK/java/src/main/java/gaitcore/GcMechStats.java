package gaitcore;

/** Per-mech telemetry snapshot populated by {@link GcContext#stats}. */
public final class GcMechStats {
    public String mode = "";
    public String gait = "";
    public int gaitOverride;
    public String state = "";
    public float duty;
    public float cycle;
    public float maxStretch;
    public float simMs;
    public int rays;
    public int climbEnabled;
    public int crouchToggle;
}
