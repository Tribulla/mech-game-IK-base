package gaitcore;

import java.util.Arrays;

/**
 * Self-contained unit, integration, and multithreaded bit-determinism test suite for GaitCore Java.
 * Mirrors {@code IK/c/tests/test_gaitcore_c.c} plus introspection, debug-line, and profiler checks.
 */
public final class GaitCoreTest {

    private static void check(boolean cond, String msg) {
        if (!cond) {
            throw new AssertionError("Assertion failed: " + msg);
        }
    }

    private static void checkNear(double actual, double expected, double eps, String msg) {
        if (Math.abs(actual - expected) > eps) {
            throw new AssertionError(msg + " (got " + actual + ", expected " + expected + ")");
        }
    }

    private static void testVmathGeometry() {
        Vec3.Vec2[] pts = {
                new Vec3.Vec2(-1, -1),
                new Vec3.Vec2(1, -1),
                new Vec3.Vec2(1, 1),
                new Vec3.Vec2(-1, 1),
                new Vec3.Vec2(0, 0),
                new Vec3.Vec2(0.5, 0.2)
        };
        Vec3.Vec2[] hull = new Vec3.Vec2[GaitCore.MAX_HULL_PTS];
        int nh = Vec3.convexHull2d(pts, pts.length, hull);
        check(nh == 4, "convexHull2d drops interior points");

        double inside = Vec3.supportMargin(hull, nh, new Vec3.Vec2(0.0, 0.0));
        double outside = Vec3.supportMargin(hull, nh, new Vec3.Vec2(2.0, 0.0));
        checkNear(inside, 1.0, 1e-6, "supportMargin inside unit square");
        checkNear(outside, -1.0, 1e-6, "supportMargin outside unit square");

        Vec3[] plPts = {
                new Vec3(0, 1.0, 0),
                new Vec3(2, 1.4, 0),
                new Vec3(0, 1.0, 2),
                new Vec3(2, 1.4, 2)
        };
        Vec3.PlaneFit fit = new Vec3.PlaneFit();
        int ok = Vec3.fitPlane(plPts, null, 4, fit);
        check(ok == 2, "fitPlane finds sloped plane normal");
        Vec3 expectedN = new Vec3(-0.2, 1.0, 0.0).norm(Vec3.WORLD_UP);
        checkNear(fit.n.dot(expectedN), 1.0, 1e-6, "fitPlane normal matches -0.2x slope");

        Vec3 arc = Vec3.turnDisplacement(new Vec3(0, 0, 10), Vec3.WORLD_UP, Math.PI);
        check(arc.len() < 7.0, "turnDisplacement shrinks chord along curved arc");
    }

    private static void testIkSolversAndLimits() {
        double[] tb = new double[4];
        boolean r2 = LegIK.solveTwoBone(1.5, 1.5, 2.0, -0.5, 1, tb);
        check(r2, "solveTwoBone reaches reachable target");
        checkNear(Math.hypot(tb[2] - 2.0, tb[3] + 0.5), 0.0, 1e-4, "solveTwoBone tip exact");
        checkNear(Math.hypot(tb[0], tb[1]), 1.5, 1e-4, "solveTwoBone thigh length preserved");
        checkNear(Math.hypot(tb[2] - tb[0], tb[3] - tb[1]), 1.5, 1e-4, "solveTwoBone shin length preserved");

        double[] lens3 = { 1.0, 1.2, 1.1 };
        int[] sides3 = { 1, -1 };
        double[][] fpts = new double[GaitCore.MAX_SEGS + 1][2];
        LegIK.initialPose2d(lens3, 3, sides3, 1.8, -0.8, fpts);
        boolean rf = LegIK.solveFabrik2d(lens3, 3, fpts, 1.8, -0.8, sides3, 12, 1e-4);
        check(rf, "solveFabrik2d reaches 3-bone target");
        checkNear(Math.hypot(fpts[3][0] - 1.8, fpts[3][1] + 0.8), 0.0, 1e-3, "solveFabrik2d tip exact");

        LegIK radial = new LegIK();
        MechDef.Limits rLim = new MechDef.Limits();
        rLim.hasCoxa = true;
        rLim.coxa = 55.0;
        rLim.hasHip = true;
        rLim.hip[0] = -45.0;
        rLim.hip[1] = 85.0;
        rLim.bendCount = 1;
        rLim.bend[0][0] = 20.0;
        rLim.bend[0][1] = 150.0;
        double[] lens2 = { 1.5, 1.7 };
        int[] sides2 = { 1 };
        radial.init(GaitCore.LEG_PLANE_RADIAL, 0.35, lens2, 2, sides2, new Vec3(1.0, 0.0, 0.0), rLim);

        Vec3.Frame frame = Vec3.LOCAL_FRAME.copy();
        Vec3 mount = new Vec3(0.8, 1.2, 0.0);
        Vec3 goodTgt = new Vec3(2.4, 0.0, 0.2);
        Vec3[] joints = new Vec3[GaitCore.MAX_SEGS + 2];
        int nj = radial.solve(mount, frame, goodTgt, 0.0, joints);
        check(nj == 4, "radial 2-bone outputs mount+coxa+knee+foot");
        check(radial.reached && !radial.limited, "radial LegIK.solve reaches good target without hitting limits");
        checkNear(joints[3].dist(goodTgt), 0.0, 0.02, "radial foot lands at target");
        check(radial.feasible(mount, frame, goodTgt, 6.0), "radial target reported feasible");

        Vec3 badTgt = new Vec3(-2.5, 0.0, 0.0);
        check(!radial.feasible(mount, frame, badTgt, 6.0), "radial target behind coxa yaw range infeasible");

        LegIK sag = new LegIK();
        MechDef.Limits sLim = new MechDef.Limits();
        sLim.hasRoll = true;
        sLim.roll = 18.0;
        sLim.hasHip = true;
        sLim.hip[0] = -70.0;
        sLim.hip[1] = 75.0;
        sLim.bendCount = 1;
        sLim.bend[0][0] = 10.0;
        sLim.bend[0][1] = 140.0;
        double[] lensSag = { 1.4, 1.6 };
        int[] sidesSag = { -1 };
        sag.init(GaitCore.LEG_PLANE_SAGITTAL, 0.0, lensSag, 2, sidesSag, new Vec3(0, -1, 0), sLim);

        Vec3 sagMount = new Vec3(0.45, 2.2, 0.0);
        Vec3 sagGood = new Vec3(0.45, 0.0, 0.2);
        sag.solve(sagMount, frame, sagGood, 0.0, joints);
        check(sag.reached && !sag.limited, "sagittal biped reaches nominal target");
        Vec3 sagBad = new Vec3(2.5, 0.0, 0.0);
        check(!sag.feasible(sagMount, frame, sagBad, 6.0), "sagittal target far outside hip roll infeasible");
    }

    private static void testGaitselAndBlend() {
        checkNear(Gait.phaseLerp(0.9, 0.1, 0.5), 0.0, 1e-6, "phaseLerp wraps across 1.0 -> 0.0");
        checkNear(Gait.phaseLerp(0.1, 0.9, 0.5), 0.0, 1e-6, "phaseLerp wraps across 0.0 -> 1.0");

        Gait[] qs = new Gait[GaitCore.MAX_GAITS];
        int nq = Gait.gaitsQuadruped(qs);
        check(nq == 6, "quadruped defines 6 gaits");
        check(Gait.find(qs, nq, "trot") == 2, "trot present in quadruped gaits");
        check(Gait.find(qs, nq, "nonexistent") == -1, "missing gait returns -1");
    }

    private static void testStubWorldObox() {
        GcStubWorld w = new GcStubWorld(0.0, true);
        w.addOBox(0.0, 1.0, 5.0, 2.0, 0.25, 4.0, 0.0, -0.35);

        GcWorldCallbacks.RayHit hit = new GcWorldCallbacks.RayHit();
        boolean ok1 = w.raycast(0.0, 6.0, 5.0, 0.0, -1.0, 0.0, 10.0, hit);
        check(ok1, "raycast hits rotated OBB");
        check(hit.ny > 0.85, "rotated OBB top normal points upward");
        check(hit.py > 0.8, "rotated OBB top surface elevated above ground");

        double[] outXyz = new double[3];
        double[] outNxyz = new double[3];
        boolean pushed = w.spherePush(0.0, 1.05, 5.0, 0.5, outXyz, outNxyz);
        check(pushed && outNxyz[1] > 0.5, "spherePush penetrates rotated OBB");
    }

    private static void testAllMechsWalkSlideJump() {
        GcStubWorld w = new GcStubWorld(0.0, true);
        try (GcContext ctx = GcContext.create(1)) {
            ctx.setWorld(w);

            int nk = GaitCore.kindCount();
            check(nk == 9, "9 built-in mech kinds");
            float[] inp = new float[GaitCore.INPUT_STRIDE];
            float[] evs = new float[128 * GaitCore.EVENT_FLOATS];

            for (int k = 0; k < nk; k++) {
                ctx.reset();
                int idx = ctx.spawn(k, 0.0f, 0.0f, 0.0f, 0.0f);
                check(idx == 0, "spawn returns handle 0");

                int nfloats = ctx.floatsPerMech(0);
                float[] pose = new float[nfloats];
                GcMechInfo info = new GcMechInfo();
                ctx.info(0, info);
                check(info.legs >= 2 && info.legs <= 8, "valid leg count");
                check(nfloats == GaitCore.HEADER_FLOATS + (info.jointsPerLeg * 3 + GaitCore.LEG_EXTRA_FLOATS) * info.legs,
                        "packed pose float stride matches header + per-leg joints/extras");

                // Phase 1: accelerate and run forward 1.5s
                Arrays.fill(inp, 0.0f);
                inp[GaitCore.IN_FWD] = 1.0f;
                inp[GaitCore.IN_RUN] = 1.0f;
                for (int f = 0; f < 90; f++) {
                    ctx.step(1.0f / 60.0f, inp, 1);
                }
                ctx.poses(pose, pose.length);
                float zRun = pose[GaitCore.POSE_POS + 2];
                check(zRun > 1.0f, info.name + " moved forward during run");

                // Phase 2: trigger quick slide + turn for 0.4s
                inp[GaitCore.IN_DRIFT] = 1.0f;
                inp[GaitCore.IN_TURN] = 0.6f;
                for (int f = 0; f < 24; f++) {
                    ctx.step(1.0f / 60.0f, inp, 1);
                }
                ctx.poses(pose, pose.length);
                check(Float.isFinite(pose[GaitCore.POSE_DRIFT]), info.name + " slide telemetry finite");
                inp[GaitCore.IN_DRIFT] = 0.0f;
                inp[GaitCore.IN_TURN] = 0.0f;

                // Phase 3: jump and observe airborne state + settle
                inp[GaitCore.IN_JUMP] = 1.0f;
                ctx.step(1.0f / 60.0f, inp, 1);
                inp[GaitCore.IN_JUMP] = 0.0f;
                boolean sawAir = false;
                for (int f = 0; f < 120; f++) {
                    ctx.step(1.0f / 60.0f, inp, 1);
                    ctx.poses(pose, pose.length);
                    if ((int) pose[GaitCore.POSE_STATE] == GaitCore.STATE_AIR) {
                        sawAir = true;
                    }
                    int nevFloats = ctx.events(evs, evs.length);
                    check(nevFloats >= 0 && nevFloats % GaitCore.EVENT_FLOATS == 0, "event float count aligned");
                    int nev = nevFloats / GaitCore.EVENT_FLOATS;
                    for (int ei = 0; ei < nev; ei++) {
                        check(Float.isFinite(evs[ei * GaitCore.EVENT_FLOATS + 2]), "event coords finite");
                    }
                }
                ctx.poses(pose, pose.length);
                check(sawAir, info.name + " entered airborne state on jump");
                for (int i = 0; i < nfloats; i++) {
                    check(Float.isFinite(pose[i]), info.name + " pose floats remain finite");
                }
                check(pose[GaitCore.POSE_POS + 1] > 0.1f && pose[GaitCore.POSE_POS + 1] < 12.0f,
                        info.name + " remained above ground and settled cleanly");
            }
        }
    }

    private static long runDetSim(
            GcStubWorld world, int threads, int mechCount, int frames,
            float[] outPoses, float[] outEvs) {
        try (GcContext ctx = GcContext.create(threads)) {
            ctx.setWorld(world);
            int nk = GaitCore.kindCount();
            for (int i = 0; i < mechCount; i++) {
                float x = (i % 8 - 3.5f) * 8.0f;
                float z = (i / 8 - 1.5f) * 8.0f;
                ctx.spawn(i % nk, x, 0.0f, z, (i * 0.4f) % 6.28f);
            }

            float[] inputs = new float[mechCount * GaitCore.INPUT_STRIDE];
            for (int f = 0; f < frames; f++) {
                for (int i = 0; i < mechCount; i++) {
                    int base = i * GaitCore.INPUT_STRIDE;
                    inputs[base + GaitCore.IN_FWD] = 1.0f;
                    inputs[base + GaitCore.IN_TURN] = (f > 60 && f < 100) ? 0.4f : 0.0f;
                    inputs[base + GaitCore.IN_RUN] = (i % 2 == 0) ? 1.0f : 0.0f;
                    inputs[base + GaitCore.IN_DRIFT] = (f > 70 && f < 90 && (i % 3 == 0)) ? 1.0f : 0.0f;
                    inputs[base + GaitCore.IN_JUMP] = (f == 100 && (i % 4 == 0)) ? 1.0f : 0.0f;
                }
                ctx.step(1.0f / 60.0f, inputs, mechCount);
            }
            int np = ctx.poses(outPoses, outPoses.length);
            int nevFloats = ctx.events(outEvs, outEvs.length);
            return (((long) np) << 32) | (nevFloats & 0xffffffffL);
        }
    }

    private static void testMultithreadDeterminism() {
        GcStubWorld w = new GcStubWorld(0.0, true);
        w.addBox(4.0, 0.0, 4.0, 12.0, 0.5, 16.0);
        w.addOBox(-8.0, 0.8, 10.0, 3.0, 0.3, 6.0, 0.0, -0.25);

        final int mechCount = 24;
        final int frames = 140;
        float[] p1 = new float[mechCount * 256];
        float[] p4 = new float[mechCount * 256];
        float[] p8 = new float[mechCount * 256];
        float[] e1 = new float[1024 * GaitCore.EVENT_FLOATS];
        float[] e4 = new float[1024 * GaitCore.EVENT_FLOATS];
        float[] e8 = new float[1024 * GaitCore.EVENT_FLOATS];

        long r1 = runDetSim(w, 1, mechCount, frames, p1, e1);
        long r4 = runDetSim(w, 4, mechCount, frames, p4, e4);
        long r8 = runDetSim(w, 8, mechCount, frames, p8, e8);
        check(r1 == r4 && r1 == r8, "1T / 4T / 8T packed pose + event counts match");

        int np = (int) (r1 >>> 32);
        int nevFloats = (int) r1;
        for (int i = 0; i < np; i++) {
            check(Float.floatToRawIntBits(p1[i]) == Float.floatToRawIntBits(p4[i]),
                    "1T vs 4T bit-exact pose float at index " + i);
            check(Float.floatToRawIntBits(p1[i]) == Float.floatToRawIntBits(p8[i]),
                    "1T vs 8T bit-exact pose float at index " + i);
        }
        for (int i = 0; i < nevFloats; i++) {
            check(Float.floatToRawIntBits(e1[i]) == Float.floatToRawIntBits(e4[i]),
                    "1T vs 4T bit-exact event float at index " + i);
            check(Float.floatToRawIntBits(e1[i]) == Float.floatToRawIntBits(e8[i]),
                    "1T vs 8T bit-exact event float at index " + i);
        }
    }

    private static void testCustomMechApi() {
        GcStubWorld w = new GcStubWorld(0.0, true);
        w.addBox(-4.0, 0.0, 3.0, 4.0, 0.45, 7.0);

        try (GcContext ctx = GcContext.create(2)) {
            ctx.setWorld(w);

            // Validation rejects zeroed/invalid descriptors
            check(ctx.registerMechType(new GcMechDesc()) == -1, "empty descriptor rejected");
            check(ctx.spawn(999, 0.0f, 0.0f, 0.0f, 0.0f) == -1, "out-of-range kind rejected");

            GcMechDesc dPreset = new GcMechDesc();
            boolean okPreset = GcMechDesc.fromPreset(2, dPreset);
            check(okPreset, "GcMechDesc.fromPreset(2) succeeds");
            check(dPreset.numLegs == 2 && "STRIDER".equals(dPreset.name), "preset 2 populates STRIDER");

            GcMechDesc custom = GcMechDesc.createDefault(4);
            custom.name = "MY_CUSTOM_SPIDER";
            custom.blurb = "Test 4-leg radial crawler";
            custom.planeMode = GaitCore.LEG_PLANE_RADIAL;
            custom.coxa = 0.35f;
            custom.numSegments = 2;
            custom.segmentLengths[0] = 1.6f;
            custom.segmentLengths[1] = 1.8f;
            custom.bendSides[0] = 1;
            custom.limits.hasCoxa = 1;
            custom.limits.coxaDeg = 55.0f;
            custom.limits.hasHip = 1;
            custom.limits.hipDeg[0] = -40.0f;
            custom.limits.hipDeg[1] = 85.0f;
            custom.limits.bendCount = 1;
            custom.limits.bendDeg[0][0] = 20.0f;
            custom.limits.bendDeg[0][1] = 148.0f;
            custom.rideHeight = 1.4f;
            custom.speedSlow = 4.0f;
            custom.speedWalk = 11.0f;
            custom.speedRun = 19.0f;
            custom.accel = 28.0f;
            custom.accelRun = 20.0f;
            custom.turnRate = 3.2f;
            custom.stepHeight = 0.75f;
            custom.stepMax = 1.0f;
            custom.mass = 1.0f;
            custom.bodyHalf[0] = 0.9f;
            custom.bodyHalf[1] = 0.4f;
            custom.bodyHalf[2] = 1.2f;
            float[][] mounts = { { -0.7f, 0.0f, 0.8f }, { 0.7f, 0.0f, 0.8f }, { -0.7f, 0.0f, -0.8f }, { 0.7f, 0.0f, -0.8f } };
            float[][] rests = { { -1.9f, 1.3f }, { 1.9f, 1.3f }, { -1.9f, -1.3f }, { 1.9f, -1.3f } };
            for (int i = 0; i < 4; i++) {
                custom.legs[i].mount[0] = mounts[i][0];
                custom.legs[i].mount[1] = mounts[i][1];
                custom.legs[i].mount[2] = mounts[i][2];
                custom.legs[i].restXz[0] = rests[i][0];
                custom.legs[i].restXz[1] = rests[i][1];
            }

            int ck = ctx.registerMechType(custom);
            check(ck == GaitCore.kindCount(), "first custom type gets index right after builtins");
            check("MY_CUSTOM_SPIDER".equals(ctx.contextKindName(ck)), "custom kind name registered");

            int h1 = ctx.spawn(ck, 0.0f, 0.0f, 0.0f, 0.0f);
            int h2 = ctx.spawnCustom(custom, 3.0f, 0.0f, 0.0f, 0.0f);
            check(h1 == 0 && h2 == 1, "spawn and spawnCustom succeed");
            check(ctx.contextKindCount() == GaitCore.kindCount() + 1, "identical spawnCustom deduplicates archetype");

            GcMechInfo info = new GcMechInfo();
            ctx.info(h2, info);
            check(info.legs == 4 && "MY_CUSTOM_SPIDER".equals(info.name), "custom info matches descriptor");

            float[] inputs = new float[2 * GaitCore.INPUT_STRIDE];
            inputs[GaitCore.IN_FWD] = 1.0f;
            inputs[GaitCore.INPUT_STRIDE + GaitCore.IN_FWD] = 1.0f;
            for (int f = 0; f < 180; f++) {
                ctx.step(1.0f / 60.0f, inputs, 2);
            }
            float[] poseBuf = new float[512];
            ctx.poses(poseBuf, poseBuf.length);
            check(poseBuf[GaitCore.POSE_POS + 2] > 6.0f,
                    "custom spider walks forward cleanly (z=" + poseBuf[GaitCore.POSE_POS + 2] + ")");

            ctx.despawnTo(0);
            ctx.clearCustomTypes();
            check(ctx.contextKindCount() == GaitCore.kindCount(), "clearCustomTypes restores builtin kind count");
        }
    }

    private static void testIntrospectionDebugAndProfile() {
        GcStubWorld w = new GcStubWorld(0.0, true);
        try (GcContext ctx = GcContext.create(0)) {
            check(ctx.getThreads() >= 1, "auto thread count >= 1");
            ctx.setThreads(2);
            check(ctx.getThreads() == 2, "setThreads updates thread count");
            ctx.setWorld(w);
            ctx.setDebug(true);

            GcBenchConfig cfg = new GcBenchConfig();
            cfg.profile = 1;
            ctx.setBenchConfig(cfg);
            ctx.resetQueryStats();

            ctx.spawn(0, 0, 0, 0, 0);
            ctx.spawn(1, 5, 0, 0, 0);
            float[] inp = new float[2 * GaitCore.INPUT_STRIDE];
            inp[GaitCore.IN_FWD] = 1.0f;
            for (int f = 0; f < 20; f++) {
                ctx.step(1.0f / 60.0f, inp, 2);
            }

            GcMechStats st = new GcMechStats();
            check(ctx.stats(0, st), "stats returns true for handle 0");
            check(!st.gait.isEmpty() && !st.state.isEmpty(), "stats populates gait and state strings");

            float[] dbg = new float[2048 * GaitCore.DEBUG_LINE_FLOATS];
            int nLines = ctx.debugLines(0, dbg, dbg.length);
            check(nLines > 0, "setDebug(true) records debug lines");

            float[] prof = new float[GaitCore.PROFILE_FLOATS];
            int np = ctx.profileData(prof, prof.length);
            check(np == GaitCore.PROFILE_FLOATS, "profileData populates full snapshot");
            check(prof[12] > 0.0f, "profileData reports rays executed");

            GcQueryStats qs = ctx.getQueryStats();
            check(qs.rayCalls > 0 && qs.pushCalls > 0, "query stats count raycalls and pushes");

            check(ctx.respawn(1, 2, -3.0f, 0.0f, 1.0f, 0.5f) == 1, "respawn replaces slot in place");
            GcMechInfo info = new GcMechInfo();
            ctx.info(1, info);
            check(info.kind == 2, "respawn changed kind to 2");

            ctx.despawnTo(1);
            check(ctx.count() == 1, "despawnTo shrinks active count");
        }
    }

    public static void main(String[] args) {
        testVmathGeometry();
        testIkSolversAndLimits();
        testGaitselAndBlend();
        testStubWorldObox();
        testAllMechsWalkSlideJump();
        testMultithreadDeterminism();
        testCustomMechApi();
        testIntrospectionDebugAndProfile();
        System.out.println("All GaitCore Java tests passed (8/8).");
    }
}
