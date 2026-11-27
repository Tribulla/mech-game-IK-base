package gaitcore;

/**
 * Runtime benchmark, LOD, and profiler settings passed to {@link GcContext#setBenchConfig}.
 * Mirrors {@code GcBenchConfig} in {@code gaitcore.h}.
 */
public final class GcBenchConfig {
    public float maxSubstep = (float) (1.0 / 60.0);
    public float legIkHz = 0.0f;   // 0 = solve IK every substep
    public float probeHz = 20.0f;
    public float speedScale = 1.0f;
    public int planeRoll = 1;
    public int profile = 0;

    public GcBenchConfig() {
    }

    public GcBenchConfig(float maxSubstep, float legIkHz, float probeHz, float speedScale, int planeRoll, int profile) {
        this.maxSubstep = maxSubstep;
        this.legIkHz = legIkHz;
        this.probeHz = probeHz;
        this.speedScale = speedScale;
        this.planeRoll = planeRoll;
        this.profile = profile;
    }

    public GcBenchConfig copy() {
        return new GcBenchConfig(maxSubstep, legIkHz, probeHz, speedScale, planeRoll, profile);
    }
}
