package gaitcore;

import java.util.Arrays;
import java.util.Objects;

/**
 * Custom mech archetype descriptor passed to {@link GcContext#registerMechType} or
 * {@link GcContext#spawnCustom}.
 *
 * <p>Keep rest reach around 50-65% of {@code (coxa + sum(segmentLengths))} so legs have room to
 * extend into holes and compress over bumps. List legs front-to-back in left/right pairs
 * ({@code L0, R0, L1, R1, ...}) so built-in gait tables align properly.
 */
public final class GcMechDesc {

    /** Joint angle limits in degrees for the planar IK solver. */
    public static final class JointLimitsDesc {
        public int hasCoxa;
        public float coxaDeg;
        public int hasRoll;
        public float rollDeg;
        public int hasHip;
        public final float[] hipDeg = new float[2];
        public int bendCount;
        public final float[][] bendDeg = new float[GaitCore.MAX_SEGS_PUBLIC - 1][2];

        public JointLimitsDesc copy() {
            JointLimitsDesc d = new JointLimitsDesc();
            d.copyFrom(this);
            return d;
        }

        public void copyFrom(JointLimitsDesc o) {
            hasCoxa = o.hasCoxa;
            coxaDeg = o.coxaDeg;
            hasRoll = o.hasRoll;
            rollDeg = o.rollDeg;
            hasHip = o.hasHip;
            hipDeg[0] = o.hipDeg[0];
            hipDeg[1] = o.hipDeg[1];
            bendCount = o.bendCount;
            for (int i = 0; i < bendDeg.length; i++) {
                bendDeg[i][0] = o.bendDeg[i][0];
                bendDeg[i][1] = o.bendDeg[i][1];
            }
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof JointLimitsDesc o)) return false;
            return hasCoxa == o.hasCoxa
                    && Float.compare(coxaDeg, o.coxaDeg) == 0
                    && hasRoll == o.hasRoll
                    && Float.compare(rollDeg, o.rollDeg) == 0
                    && hasHip == o.hasHip
                    && Arrays.equals(hipDeg, o.hipDeg)
                    && bendCount == o.bendCount
                    && Arrays.deepEquals(bendDeg, o.bendDeg);
        }

        @Override
        public int hashCode() {
            int h = Objects.hash(hasCoxa, coxaDeg, hasRoll, rollDeg, hasHip, bendCount);
            h = 31 * h + Arrays.hashCode(hipDeg);
            h = 31 * h + Arrays.deepHashCode(bendDeg);
            return h;
        }
    }

    /** Body-local hip mount and neutral ground target offsets for one leg. */
    public static final class LegDesc {
        public final float[] mount = new float[3];   // [right, up, fwd] in meters
        public final float[] restXz = new float[2];  // [right, fwd] in meters
        public int side;                             // -1 left, +1 right, 0 infer from X

        public LegDesc copy() {
            LegDesc l = new LegDesc();
            l.copyFrom(this);
            return l;
        }

        public void copyFrom(LegDesc o) {
            System.arraycopy(o.mount, 0, mount, 0, 3);
            System.arraycopy(o.restXz, 0, restXz, 0, 2);
            side = o.side;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof LegDesc o)) return false;
            return side == o.side
                    && Arrays.equals(mount, o.mount)
                    && Arrays.equals(restXz, o.restXz);
        }

        @Override
        public int hashCode() {
            int h = Arrays.hashCode(mount);
            h = 31 * h + Arrays.hashCode(restXz);
            return 31 * h + side;
        }
    }

    public String name = "";
    public String blurb = "";
    public int faction;
    public int planeMode; // LEG_PLANE_RADIAL or LEG_PLANE_SAGITTAL
    public float coxa;
    public int numSegments;
    public final float[] segmentLengths = new float[GaitCore.MAX_SEGS_PUBLIC];
    public final int[] bendSides = new int[GaitCore.MAX_SEGS_PUBLIC - 1];
    public final float[] segmentWidths = new float[GaitCore.MAX_SEGS_PUBLIC];
    public final JointLimitsDesc limits = new JointLimitsDesc();
    public int numLegs;
    public final LegDesc[] legs = new LegDesc[GaitCore.MAX_LEGS_PUBLIC];
    public final float[] bodyHalf = new float[3];
    public float rideHeight;
    public float speedSlow;
    public float speedWalk;
    public float speedRun;
    public float accel;
    public float accelRun;
    public float turnRate;
    public float stepHeight;
    public float stepMax;
    public float mantleMax;
    public float wireMax;
    public int climber;
    public float jumpSpeed;
    public float maxTiltDeg;
    public float lookTiltDeg;
    public float mass;
    public float bound;
    public float boundPitchDeg;
    public float zone;
    public float swingMin;
    public float swingMax;
    public float size;

    public GcMechDesc() {
        for (int i = 0; i < legs.length; i++) {
            legs[i] = new LegDesc();
        }
    }

    public void clear() {
        name = "";
        blurb = "";
        faction = 0;
        planeMode = 0;
        coxa = 0.0f;
        numSegments = 0;
        Arrays.fill(segmentLengths, 0.0f);
        Arrays.fill(bendSides, 0);
        Arrays.fill(segmentWidths, 0.0f);
        limits.hasCoxa = 0;
        limits.coxaDeg = 0.0f;
        limits.hasRoll = 0;
        limits.rollDeg = 0.0f;
        limits.hasHip = 0;
        Arrays.fill(limits.hipDeg, 0.0f);
        limits.bendCount = 0;
        for (float[] b : limits.bendDeg) {
            Arrays.fill(b, 0.0f);
        }
        numLegs = 0;
        for (LegDesc l : legs) {
            Arrays.fill(l.mount, 0.0f);
            Arrays.fill(l.restXz, 0.0f);
            l.side = 0;
        }
        Arrays.fill(bodyHalf, 0.0f);
        rideHeight = 0.0f;
        speedSlow = 0.0f;
        speedWalk = 0.0f;
        speedRun = 0.0f;
        accel = 0.0f;
        accelRun = 0.0f;
        turnRate = 0.0f;
        stepHeight = 0.0f;
        stepMax = 0.0f;
        mantleMax = 0.0f;
        wireMax = 0.0f;
        climber = 0;
        jumpSpeed = 0.0f;
        maxTiltDeg = 0.0f;
        lookTiltDeg = 0.0f;
        mass = 0.0f;
        bound = 0.0f;
        boundPitchDeg = 0.0f;
        zone = 0.0f;
        swingMin = 0.0f;
        swingMax = 0.0f;
        size = 0.0f;
    }

    /** Populates valid defaults for 2, 4, 6, or 8 legs. */
    public void initDefault(int numLegs) {
        if (numLegs <= 2) {
            MechDef.descFromPreset(2, this);
            this.name = "BIPED";
            this.blurb = "Default 2-legged sagittal walker";
        } else if (numLegs <= 4) {
            MechDef.descFromPreset(0, this);
            this.name = "QUADRUPED";
        } else if (numLegs <= 6) {
            MechDef.descFromPreset(7, this);
            this.name = "HEXAPOD";
        } else {
            MechDef.descFromPreset(5, this);
            this.name = "OCTOPOD";
        }
    }

    /** Factory returning a valid default descriptor for 2, 4, 6, or 8 legs. */
    public static GcMechDesc createDefault(int numLegs) {
        GcMechDesc d = new GcMechDesc();
        d.initDefault(numLegs);
        return d;
    }

    /** Copies a built-in preset (0..kindCount()-1) into {@code outDesc}. */
    public static boolean fromPreset(int presetKind, GcMechDesc outDesc) {
        return MechDef.descFromPreset(presetKind, outDesc);
    }

    public GcMechDesc copy() {
        GcMechDesc d = new GcMechDesc();
        d.copyFrom(this);
        return d;
    }

    public void copyFrom(GcMechDesc o) {
        name = o.name;
        blurb = o.blurb;
        faction = o.faction;
        planeMode = o.planeMode;
        coxa = o.coxa;
        numSegments = o.numSegments;
        System.arraycopy(o.segmentLengths, 0, segmentLengths, 0, GaitCore.MAX_SEGS_PUBLIC);
        System.arraycopy(o.bendSides, 0, bendSides, 0, GaitCore.MAX_SEGS_PUBLIC - 1);
        System.arraycopy(o.segmentWidths, 0, segmentWidths, 0, GaitCore.MAX_SEGS_PUBLIC);
        limits.copyFrom(o.limits);
        numLegs = o.numLegs;
        for (int i = 0; i < GaitCore.MAX_LEGS_PUBLIC; i++) {
            legs[i].copyFrom(o.legs[i]);
        }
        System.arraycopy(o.bodyHalf, 0, bodyHalf, 0, 3);
        rideHeight = o.rideHeight;
        speedSlow = o.speedSlow;
        speedWalk = o.speedWalk;
        speedRun = o.speedRun;
        accel = o.accel;
        accelRun = o.accelRun;
        turnRate = o.turnRate;
        stepHeight = o.stepHeight;
        stepMax = o.stepMax;
        mantleMax = o.mantleMax;
        wireMax = o.wireMax;
        climber = o.climber;
        jumpSpeed = o.jumpSpeed;
        maxTiltDeg = o.maxTiltDeg;
        lookTiltDeg = o.lookTiltDeg;
        mass = o.mass;
        bound = o.bound;
        boundPitchDeg = o.boundPitchDeg;
        zone = o.zone;
        swingMin = o.swingMin;
        swingMax = o.swingMax;
        size = o.size;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof GcMechDesc o)) return false;
        return Objects.equals(name, o.name)
                && Objects.equals(blurb, o.blurb)
                && faction == o.faction
                && planeMode == o.planeMode
                && Float.compare(coxa, o.coxa) == 0
                && numSegments == o.numSegments
                && Arrays.equals(segmentLengths, o.segmentLengths)
                && Arrays.equals(bendSides, o.bendSides)
                && Arrays.equals(segmentWidths, o.segmentWidths)
                && Objects.equals(limits, o.limits)
                && numLegs == o.numLegs
                && Arrays.equals(legs, o.legs)
                && Arrays.equals(bodyHalf, o.bodyHalf)
                && Float.compare(rideHeight, o.rideHeight) == 0
                && Float.compare(speedSlow, o.speedSlow) == 0
                && Float.compare(speedWalk, o.speedWalk) == 0
                && Float.compare(speedRun, o.speedRun) == 0
                && Float.compare(accel, o.accel) == 0
                && Float.compare(accelRun, o.accelRun) == 0
                && Float.compare(turnRate, o.turnRate) == 0
                && Float.compare(stepHeight, o.stepHeight) == 0
                && Float.compare(stepMax, o.stepMax) == 0
                && Float.compare(mantleMax, o.mantleMax) == 0
                && Float.compare(wireMax, o.wireMax) == 0
                && climber == o.climber
                && Float.compare(jumpSpeed, o.jumpSpeed) == 0
                && Float.compare(maxTiltDeg, o.maxTiltDeg) == 0
                && Float.compare(lookTiltDeg, o.lookTiltDeg) == 0
                && Float.compare(mass, o.mass) == 0
                && Float.compare(bound, o.bound) == 0
                && Float.compare(boundPitchDeg, o.boundPitchDeg) == 0
                && Float.compare(zone, o.zone) == 0
                && Float.compare(swingMin, o.swingMin) == 0
                && Float.compare(swingMax, o.swingMax) == 0
                && Float.compare(size, o.size) == 0;
    }

    @Override
    public int hashCode() {
        int h = Objects.hash(name, blurb, faction, planeMode, coxa, numSegments, limits, numLegs,
                rideHeight, speedSlow, speedWalk, speedRun, accel, accelRun, turnRate,
                stepHeight, stepMax, mantleMax, wireMax, climber, jumpSpeed, maxTiltDeg,
                lookTiltDeg, mass, bound, boundPitchDeg, zone, swingMin, swingMax, size);
        h = 31 * h + Arrays.hashCode(segmentLengths);
        h = 31 * h + Arrays.hashCode(bendSides);
        h = 31 * h + Arrays.hashCode(segmentWidths);
        h = 31 * h + Arrays.hashCode(legs);
        h = 31 * h + Arrays.hashCode(bodyHalf);
        return h;
    }
}
