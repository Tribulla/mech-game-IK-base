package gaitcore;

/**
 * Read-only, thread-safe world query interface implemented by the host engine.
 *
 * <p>Worker threads invoke these methods concurrently inside {@link GcContext#step} with no
 * internal locks held. Filter out the querying mech's own collider so feet do not step on their
 * own hull, and ignore ray intersections at {@code t <= 0} so underground probes can escape cleanly.
 */
public interface GcWorldCallbacks {

    /** Caller-owned ray hit struct populated by {@link #raycast}. */
    final class RayHit {
        public double t;
        public double px;
        public double py;
        public double pz;
        public double nx;
        public double ny;
        public double nz;
        public int surface;
    }

    /**
     * Casts a unit-direction ray against static world geometry within {@code [0, maxDist]}.
     *
     * @return {@code true} if an intersection was found and written into {@code outHit}
     */
    boolean raycast(
            double ox, double oy, double oz,
            double dx, double dy, double dz,
            double maxDist,
            RayHit outHit);

    /**
     * Resolves overlap between sphere {@code (cx, cy, cz, radius)} and static world colliders.
     * Writes pushed sphere center to {@code outXyz[0..2]} and unit push normal to {@code outNxyz[0..2]}.
     * Always populate {@code outXyz} even when returning {@code false}.
     *
     * @return {@code true} if the sphere penetrated geometry
     */
    boolean spherePush(
            double cx, double cy, double cz,
            double radius,
            double[] outXyz,
            double[] outNxyz);

    /** Optional fast heightfield lookup at world {@code (x, z)}. */
    default double terrainHeight(double x, double z) {
        return 0.0;
    }
}
