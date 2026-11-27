package gaitcore;

import java.util.Arrays;

/**
 * Per-mech world query context and lock-free scratch storage.
 * Mirrors {@code GcWorldQueryCtx} in {@code gc_world.c} so workers record stats and debug rays
 * without shared synchronization or per-query heap allocations.
 */
final class WorldQueryCtx {

    static final class Hit {
        double t;
        Vec3 point = Vec3.ZERO;
        Vec3 normal = Vec3.WORLD_UP;
        int surface;
    }

    static final class PushResult {
        Vec3 center = Vec3.ZERO;
        Vec3 normal = Vec3.WORLD_UP;
    }

    GcWorldCallbacks cb;
    boolean record;
    boolean profileQueries;
    int rayCount;
    int pushCount;
    int heightCount;
    double rayMs;
    double pushMs;
    double heightMs;
    float[] rays = new float[0];
    int rayFloats;

    final GcWorldCallbacks.RayHit scratchRayHit = new GcWorldCallbacks.RayHit();
    final double[] scratchXyz = new double[3];
    final double[] scratchNxyz = new double[3];

    WorldQueryCtx(GcWorldCallbacks cb) {
        this.cb = cb;
    }

    void clearRays() {
        rayCount = 0;
        pushCount = 0;
        heightCount = 0;
        rayMs = 0.0;
        pushMs = 0.0;
        heightMs = 0.0;
        rayFloats = 0;
    }

    private void pushRay(Vec3 o, Vec3 end, double hit) {
        if (rayFloats + 7 > rays.length) {
            int newCap = rays.length > 0 ? rays.length * 2 : 512 * 7;
            rays = Arrays.copyOf(rays, newCap);
        }
        rays[rayFloats] = (float) o.x;
        rays[rayFloats + 1] = (float) o.y;
        rays[rayFloats + 2] = (float) o.z;
        rays[rayFloats + 3] = (float) end.x;
        rays[rayFloats + 4] = (float) end.y;
        rays[rayFloats + 5] = (float) end.z;
        rays[rayFloats + 6] = (float) hit;
        rayFloats += 7;
    }

    boolean raycast(Vec3 origin, Vec3 dir, double maxDist, Hit outHit) {
        rayCount += 1;
        boolean hit = false;
        if (cb != null) {
            double t0 = profileQueries ? GaitCore.timeSec() : 0.0;
            hit = cb.raycast(origin.x, origin.y, origin.z, dir.x, dir.y, dir.z, maxDist, scratchRayHit);
            if (profileQueries) {
                rayMs += (GaitCore.timeSec() - t0) * 1000.0;
            }
        }
        if (hit) {
            outHit.t = scratchRayHit.t;
            outHit.point = new Vec3(scratchRayHit.px, scratchRayHit.py, scratchRayHit.pz);
            outHit.normal = new Vec3(scratchRayHit.nx, scratchRayHit.ny, scratchRayHit.nz);
            outHit.surface = scratchRayHit.surface;
        }
        if (record) {
            Vec3 end = hit ? outHit.point : origin.add(dir.mul(maxDist));
            pushRay(origin, end, hit ? 1.0 : 0.0);
        }
        return hit;
    }

    boolean spherePush(Vec3 center, double radius, PushResult out) {
        pushCount += 1;
        scratchXyz[0] = center.x;
        scratchXyz[1] = center.y;
        scratchXyz[2] = center.z;
        scratchNxyz[0] = 0.0;
        scratchNxyz[1] = 1.0;
        scratchNxyz[2] = 0.0;
        boolean hit = false;
        if (cb != null) {
            double t0 = profileQueries ? GaitCore.timeSec() : 0.0;
            hit = cb.spherePush(center.x, center.y, center.z, radius, scratchXyz, scratchNxyz);
            if (profileQueries) {
                pushMs += (GaitCore.timeSec() - t0) * 1000.0;
            }
        }
        out.center = new Vec3(scratchXyz[0], scratchXyz[1], scratchXyz[2]);
        if (hit) {
            out.normal = new Vec3(scratchNxyz[0], scratchNxyz[1], scratchNxyz[2]);
        }
        return hit;
    }
}
