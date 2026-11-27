package gaitcore;

import java.util.Arrays;

/**
 * Gait timing library for 2-, 4-, 6-, and 8-legged walkers.
 * Mirrors {@code gc_gaits.c}. Legs are ordered front-to-back in left/right pairs (L0, R0, L1, R1, ...).
 */
final class Gait {

    String name = "";
    int nLegs;
    final double[] offsets = new double[GaitCore.MAX_LEGS];
    double duty;
    double stride;
    double cycleMin;
    double cycleMax;
    int minSupport;
    boolean adjacentLock;
    boolean staticStable;
    double lift;
    boolean pairSync;
    double reach;
    double leap;
    double stridePitch;
    int groupCount;
    final int[] groupSizes = new int[GaitCore.MAX_GROUPS];
    final int[][] groups = new int[GaitCore.MAX_GROUPS][GaitCore.MAX_GROUP_SIZE];
    boolean hasDutyFast;
    double dutyFast;
    boolean phaseLock;

    Gait copy() {
        Gait g = new Gait();
        g.copyFrom(this);
        return g;
    }

    void copyFrom(Gait o) {
        name = o.name;
        nLegs = o.nLegs;
        System.arraycopy(o.offsets, 0, offsets, 0, GaitCore.MAX_LEGS);
        duty = o.duty;
        stride = o.stride;
        cycleMin = o.cycleMin;
        cycleMax = o.cycleMax;
        minSupport = o.minSupport;
        adjacentLock = o.adjacentLock;
        staticStable = o.staticStable;
        lift = o.lift;
        pairSync = o.pairSync;
        reach = o.reach;
        leap = o.leap;
        stridePitch = o.stridePitch;
        groupCount = o.groupCount;
        System.arraycopy(o.groupSizes, 0, groupSizes, 0, GaitCore.MAX_GROUPS);
        for (int i = 0; i < GaitCore.MAX_GROUPS; i++) {
            System.arraycopy(o.groups[i], 0, groups[i], 0, GaitCore.MAX_GROUP_SIZE);
        }
        hasDutyFast = o.hasDutyFast;
        dutyFast = o.dutyFast;
        phaseLock = o.phaseLock;
    }

    private void setPairSyncGroups() {
        int pairs = nLegs / 2;
        groupCount = pairs;
        for (int k = 0; k < pairs; k++) {
            groupSizes[k] = 2;
            groups[k][0] = 2 * k;
            groups[k][1] = 2 * k + 1;
        }
    }

    private static Gait makeGait(
            String name, int nLegs, double[] offsets, double duty,
            double stride, double cycleMin, double cycleMax, int minSupport,
            boolean adjacentLock, boolean staticStable, double lift, boolean pairSync,
            double reach, double leap, double stridePitch, boolean hasDutyFast,
            double dutyFast, boolean phaseLock) {
        Gait g = new Gait();
        g.name = name;
        g.nLegs = nLegs;
        System.arraycopy(offsets, 0, g.offsets, 0, nLegs);
        g.duty = duty;
        g.stride = stride;
        g.cycleMin = cycleMin;
        g.cycleMax = cycleMax;
        g.minSupport = minSupport;
        g.adjacentLock = adjacentLock;
        g.staticStable = staticStable;
        g.lift = lift;
        g.pairSync = pairSync;
        g.reach = reach;
        g.leap = leap;
        g.stridePitch = stridePitch;
        g.hasDutyFast = hasDutyFast;
        g.dutyFast = dutyFast;
        g.phaseLock = phaseLock;
        if (pairSync) {
            g.setPairSyncGroups();
        }
        return g;
    }

    static int gaitsBiped(Gait[] out) {
        double[] o0 = { 0.0, 0.5 };
        out[0] = makeGait("stalk", 2, o0, 0.78, 1.2, 1.2, 1.8, 1, true, false, 0.7, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[1] = makeGait("walk", 2, o0, 0.62, 2.3, 0.85, 1.25, 1, true, false, 1.0, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[2] = makeGait("run", 2, o0, 0.36, 3.8, 0.52, 0.72, 0, false, false, 1.3, false, 1.0, 1.0, 1.0, false, 0.0, false);
        return 3;
    }

    static int gaitsQuadruped(Gait[] out) {
        double[] oCrawl = { 0.75, 0.25, 0.0, 0.5 };
        double[] oTrot = { 0.0, 0.5, 0.5, 0.0 };
        double[] oGallop = { 0.5, 0.6, 0.0, 0.1 };
        double[] oBound = { 0.6, 0.6, 0.35, 0.35 };

        out[0] = makeGait("crawl", 4, oCrawl, 0.82, 1.1, 1.3, 2.0, 3, true, true, 0.8, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[1] = makeGait("walk", 4, oCrawl, 0.66, 1.9, 0.9, 1.4, 2, true, false, 1.0, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[2] = makeGait("trot", 4, oTrot, 0.50, 2.8, 0.55, 0.85, 2, false, false, 1.1, false, 1.0, 1.0, 1.0, false, 0.0, false);

        out[3] = makeGait("trot_run", 4, oTrot, 0.5, 3.0, 0.34, 0.6, 0, false, false, 1.2, false, 1.6, 0.06, 0.0, true, 0.27, true);
        out[3].groupCount = 2;
        out[3].groupSizes[0] = 2; out[3].groups[0][0] = 0; out[3].groups[0][1] = 3;
        out[3].groupSizes[1] = 2; out[3].groups[1][0] = 1; out[3].groups[1][1] = 2;

        out[4] = makeGait("gallop", 4, oGallop, 0.36, 4.0, 0.42, 0.6, 0, false, false, 1.3, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[5] = makeGait("bound", 4, oBound, 0.4, 3.4, 0.5, 0.85, 0, false, false, 1.6, true, 2.2, 1.0, 1.0, false, 0.0, false);
        return 6;
    }

    static int gaitsHexapod(Gait[] out) {
        double[] oWave = { 0.667, 0.167, 0.833, 0.333, 0.0, 0.5 };
        double[] oRipple = { 0.333, 0.833, 0.667, 0.167, 0.0, 0.5 };
        double[] oTripod = { 0.0, 0.5, 0.5, 0.0, 0.0, 0.5 };
        double[] oBound = { 0.6, 0.6, 0.48, 0.48, 0.35, 0.35 };

        out[0] = makeGait("wave", 6, oWave, 0.86, 0.9, 1.0, 1.8, 5, true, true, 0.8, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[1] = makeGait("ripple", 6, oRipple, 0.66, 1.3, 0.6, 1.1, 4, true, false, 1.0, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[2] = makeGait("tripod", 6, oTripod, 0.50, 1.8, 0.34, 0.6, 3, false, false, 1.1, false, 1.0, 1.0, 1.0, false, 0.0, false);

        out[3] = makeGait("tripod_run", 6, oTripod, 0.5, 2.6, 0.32, 0.55, 0, false, false, 1.2, false, 1.6, 0.06, 0.0, true, 0.3, true);
        out[3].groupCount = 2;
        out[3].groupSizes[0] = 3; out[3].groups[0][0] = 0; out[3].groups[0][1] = 3; out[3].groups[0][2] = 4;
        out[3].groupSizes[1] = 3; out[3].groups[1][0] = 1; out[3].groups[1][1] = 2; out[3].groups[1][2] = 5;

        out[4] = makeGait("bound", 6, oBound, 0.4, 2.8, 0.45, 0.8, 0, false, false, 1.5, true, 2.0, 1.0, 1.0, false, 0.0, false);
        return 5;
    }

    static int gaitsOctopod(Gait[] out) {
        double[] oWave = { 0.625, 0.125, 0.75, 0.25, 0.875, 0.375, 0.0, 0.5 };
        double[] oMeta = { 0.25, 0.75, 0.5, 0.0, 0.75, 0.25, 0.0, 0.5 };
        double[] oTetra = { 0.0, 0.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.0 };
        double[] oBound = { 0.6, 0.6, 0.52, 0.52, 0.44, 0.44, 0.35, 0.35 };

        out[0] = makeGait("wave", 8, oWave, 0.86, 1.0, 1.1, 2.0, 6, true, true, 0.8, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[1] = makeGait("metachronal", 8, oMeta, 0.70, 1.5, 0.6, 1.2, 5, true, false, 1.0, false, 1.0, 1.0, 1.0, false, 0.0, false);
        out[2] = makeGait("tetrapod", 8, oTetra, 0.50, 2.0, 0.36, 0.65, 4, false, false, 1.1, false, 1.0, 1.0, 1.0, false, 0.0, false);

        out[3] = makeGait("tetrapod_run", 8, oTetra, 0.5, 2.4, 0.4, 0.6, 0, false, false, 1.15, false, 1.5, 0.1, 0.0, true, 0.32, true);
        out[3].groupCount = 2;
        out[3].groupSizes[0] = 4; out[3].groups[0][0] = 0; out[3].groups[0][1] = 3; out[3].groups[0][2] = 4; out[3].groups[0][3] = 7;
        out[3].groupSizes[1] = 4; out[3].groups[1][0] = 1; out[3].groups[1][1] = 2; out[3].groups[1][2] = 5; out[3].groups[1][3] = 6;

        out[4] = makeGait("bound", 8, oBound, 0.4, 2.4, 0.45, 0.8, 0, false, false, 1.3, true, 1.8, 1.0, 1.0, false, 0.0, false);
        return 5;
    }

    static int find(Gait[] gaits, int count, String name) {
        if (name == null) {
            return -1;
        }
        for (int i = 0; i < count; i++) {
            if (name.equals(gaits[i].name)) {
                return i;
            }
        }
        return -1;
    }

    /** Interpolates normalized phase offsets along the shortest wrap-around path on [0, 1). */
    static double phaseLerp(double a, double b, double t) {
        double d = GaitCore.mod1(b - a + 0.5) - 0.5;
        return GaitCore.mod1(a + d * t);
    }
}
