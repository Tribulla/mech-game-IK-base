package gaitcore;

import java.util.Arrays;

/**
 * Compiled mech archetype definitions, built-in 9-mech roster, and descriptor conversion.
 * Mirrors {@code gc_mechdefs.c}.
 */
final class MechDef {

    static final int BUILTIN_COUNT = 9;

    static final class Limits {
        boolean hasCoxa;
        double coxa;
        boolean hasRoll;
        double roll;
        boolean hasHip;
        final double[] hip = new double[2];
        int bendCount;
        final double[][] bend = new double[GaitCore.MAX_SEGS - 1][2];

        Limits copy() {
            Limits c = new Limits();
            c.hasCoxa = hasCoxa;
            c.coxa = coxa;
            c.hasRoll = hasRoll;
            c.roll = roll;
            c.hasHip = hasHip;
            c.hip[0] = hip[0];
            c.hip[1] = hip[1];
            c.bendCount = bendCount;
            for (int i = 0; i < bend.length; i++) {
                c.bend[i][0] = bend[i][0];
                c.bend[i][1] = bend[i][1];
            }
            return c;
        }
    }

    static final class LegDef {
        final double[] mount = new double[3];
        final double[] rest = new double[2];
        boolean hasSides;
        final int[] sides = new int[GaitCore.MAX_SEGS - 1];
        int side;
        boolean hasLimits;
        final Limits limits = new Limits();
    }

    String name = "";
    int kindId;
    int faction;
    String blurb = "";
    int plane;
    double coxa;
    int nSegs;
    final double[] lengths = new double[GaitCore.MAX_SEGS];
    final int[] sides = new int[GaitCore.MAX_SEGS - 1];
    final double[] widths = new double[GaitCore.MAX_SEGS];
    final Limits limits = new Limits();
    int nLegs;
    final LegDef[] legs = new LegDef[GaitCore.MAX_LEGS];
    final double[] bodyHalf = new double[3];
    double ride;
    double speedSlow;
    double speedWalk;
    double speedRun;
    double accel;
    double accelRun;
    double turnRate;
    int gaitCount;
    final Gait[] gaits = new Gait[GaitCore.MAX_GAITS];
    int gaitSlowIdx;
    int gaitWalkIdx;
    int gaitRunIdx;
    int gaitWalkFastIdx;
    int gaitDriftIdx;
    double stepHeight;
    double stepMax;
    double mantleMax;
    double wireMax;
    boolean climber;
    double jumpSpeed;
    double maxTilt;
    double lookTilt;
    double mass;
    boolean biped;
    double bound;
    double boundPitch;
    double zone;
    double swingMin;
    double swingMax;
    double size;

    MechDef() {
        for (int i = 0; i < legs.length; i++) {
            legs[i] = new LegDef();
        }
        for (int i = 0; i < gaits.length; i++) {
            gaits[i] = new Gait();
        }
    }

    private void addPair(double mx, double my, double mz, double rx, double rz) {
        int k = nLegs;
        LegDef l = legs[k];
        l.mount[0] = mx; l.mount[1] = my; l.mount[2] = mz;
        l.rest[0] = rx; l.rest[1] = rz;
        l.side = -1;

        LegDef r = legs[k + 1];
        r.mount[0] = -mx; r.mount[1] = my; r.mount[2] = mz;
        r.rest[0] = -rx; r.rest[1] = rz;
        r.side = 1;

        nLegs = k + 2;
    }

    static final MechDef[] BUILTINS = buildBuiltins();

    private static MechDef[] buildBuiltins() {
        MechDef[] defs = new MechDef[BUILTIN_COUNT];

        // 0: LYNX
        {
            MechDef d = new MechDef();
            d.name = "LYNX";
            d.kindId = 0;
            d.faction = 0;
            d.blurb = "Light recon quadruped - agile spider legs, wire anchors";
            d.plane = GaitCore.PLANE_RADIAL;
            d.coxa = 0.25;
            d.nSegs = 2;
            d.lengths[0] = 1.4; d.lengths[1] = 1.6;
            d.sides[0] = 1;
            d.widths[0] = 0.22; d.widths[1] = 0.16;
            d.limits.hasCoxa = true; d.limits.coxa = 75.0;
            d.limits.hasHip = true; d.limits.hip[0] = -30.0; d.limits.hip[1] = 75.0;
            d.limits.bendCount = 1; d.limits.bend[0][0] = 8.0; d.limits.bend[0][1] = 150.0;
            d.addPair(-0.75, 0.0, 0.95, -2.0, 2.1);
            d.addPair(-0.75, 0.0, -0.95, -2.0, -2.0);
            d.bodyHalf[0] = 0.8; d.bodyHalf[1] = 0.45; d.bodyHalf[2] = 1.25;
            d.ride = 1.15;
            d.speedSlow = 10.5; d.speedWalk = 27.3; d.speedRun = 35.7;
            d.accel = 8.0; d.accelRun = 24.0; d.turnRate = 3.4;
            d.gaitCount = Gait.gaitsQuadruped(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "crawl");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "walk");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "trot_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "trot");
            d.stepHeight = 0.45; d.stepMax = 0.7; d.mantleMax = 1.6; d.wireMax = 9.0;
            d.climber = false; d.jumpSpeed = 14.1; d.maxTilt = 20.0; d.lookTilt = 25.0;
            d.mass = 1.0; d.biped = false; d.bound = 0.45; d.boundPitch = 8.0;
            d.zone = 1.0; d.swingMin = 0.14; d.swingMax = 0.42; d.size = 1.0;
            defs[0] = d;
        }

        // 1: STRIKER
        {
            MechDef d = new MechDef();
            d.name = "STRIKER";
            d.kindId = 1;
            d.faction = 0;
            d.blurb = "Assault quadruped - 3-seg legs, high jump, wire anchors";
            d.plane = GaitCore.PLANE_RADIAL;
            d.coxa = 0.3;
            d.nSegs = 3;
            d.lengths[0] = 1.6; d.lengths[1] = 1.5; d.lengths[2] = 0.55;
            d.sides[0] = 1; d.sides[1] = 1;
            d.widths[0] = 0.26; d.widths[1] = 0.2; d.widths[2] = 0.18;
            d.limits.hasCoxa = true; d.limits.coxa = 75.0;
            d.limits.hasHip = true; d.limits.hip[0] = -35.0; d.limits.hip[1] = 70.0;
            d.limits.bendCount = 2;
            d.limits.bend[0][0] = 5.0; d.limits.bend[0][1] = 145.0;
            d.limits.bend[1][0] = 15.0; d.limits.bend[1][1] = 150.0;
            d.addPair(-0.85, 0.05, 1.1, -2.3, 2.5);
            d.addPair(-0.85, 0.05, -1.1, -2.3, -2.3);
            d.bodyHalf[0] = 0.9; d.bodyHalf[1] = 0.5; d.bodyHalf[2] = 1.45;
            d.ride = 1.45;
            d.speedSlow = 14.0; d.speedWalk = 36.4; d.speedRun = 47.6;
            d.accel = 12.0; d.accelRun = 36.0; d.turnRate = 4.2;
            d.gaitCount = Gait.gaitsQuadruped(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "crawl");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "walk");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "trot_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "trot");
            d.stepHeight = 0.55; d.stepMax = 0.8; d.mantleMax = 2.2; d.wireMax = 10.0;
            d.climber = false; d.jumpSpeed = 17.9; d.maxTilt = 22.0; d.lookTilt = 25.0;
            d.mass = 1.0; d.biped = false; d.bound = 0.75; d.boundPitch = 10.0;
            d.zone = 1.15; d.swingMin = 0.12; d.swingMax = 0.4; d.size = 1.0;
            defs[1] = d;
        }

        // 2: STRIDER
        {
            MechDef d = new MechDef();
            d.name = "STRIDER";
            d.kindId = 2;
            d.faction = 0;
            d.blurb = "Combat biped - sagittal 2-leg strider with shoulder cannons";
            d.plane = GaitCore.PLANE_SAGITTAL;
            d.coxa = 0.0;
            d.nSegs = 2;
            d.lengths[0] = 1.35; d.lengths[1] = 1.35;
            d.sides[0] = 1;
            d.widths[0] = 0.26; d.widths[1] = 0.22;
            d.limits.hasRoll = true; d.limits.roll = 30.0;
            d.limits.hasHip = true; d.limits.hip[0] = -75.0; d.limits.hip[1] = 105.0;
            d.limits.bendCount = 1; d.limits.bend[0][0] = 8.0; d.limits.bend[0][1] = 150.0;
            d.addPair(-0.48, -0.25, 0.0, -0.48, 0.0);
            d.bodyHalf[0] = 0.55; d.bodyHalf[1] = 0.55; d.bodyHalf[2] = 0.55;
            d.ride = 1.95;
            d.speedSlow = 3.5; d.speedWalk = 6.5; d.speedRun = 13.5;
            d.accel = 9.0; d.accelRun = 20.0; d.turnRate = 3.6;
            d.gaitCount = Gait.gaitsBiped(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "stalk");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "walk");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "run");
            d.gaitWalkFastIdx = -1;
            d.gaitDriftIdx = d.gaitWalkIdx;
            d.stepHeight = 0.45; d.stepMax = 0.75; d.mantleMax = 1.6; d.wireMax = 0.0;
            d.climber = false; d.jumpSpeed = 12.5; d.maxTilt = 16.0; d.lookTilt = 20.0;
            d.mass = 1.0; d.biped = true; d.bound = 0.22; d.boundPitch = 3.0;
            d.zone = 0.9; d.swingMin = 0.12; d.swingMax = 0.36; d.size = 1.0;
            defs[2] = d;
        }

        // 3: GECKO
        {
            MechDef d = new MechDef();
            d.name = "GECKO";
            d.kindId = 3;
            d.faction = 0;
            d.blurb = "Wall-climbing spider scout - surface-adhesion quadruped";
            d.plane = GaitCore.PLANE_RADIAL;
            d.coxa = 0.22;
            d.nSegs = 2;
            d.lengths[0] = 1.35; d.lengths[1] = 1.50;
            d.sides[0] = 1;
            d.widths[0] = 0.20; d.widths[1] = 0.14;
            d.limits.hasCoxa = true; d.limits.coxa = 75.0;
            d.limits.hasHip = true; d.limits.hip[0] = -35.0; d.limits.hip[1] = 80.0;
            d.limits.bendCount = 1; d.limits.bend[0][0] = 8.0; d.limits.bend[0][1] = 155.0;
            d.addPair(-0.68, -0.05, 0.85, -1.85, 1.85);
            d.addPair(-0.68, -0.05, -0.85, -1.85, -1.80);
            d.bodyHalf[0] = 0.72; d.bodyHalf[1] = 0.36; d.bodyHalf[2] = 1.05;
            d.ride = 1.00;
            d.speedSlow = 10.0; d.speedWalk = 24.0; d.speedRun = 32.0;
            d.accel = 10.0; d.accelRun = 28.0; d.turnRate = 4.2;
            d.gaitCount = Gait.gaitsQuadruped(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "crawl");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "walk");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "trot_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "trot");
            d.stepHeight = 0.45; d.stepMax = 0.75; d.mantleMax = 1.6; d.wireMax = 0.0;
            d.climber = true; d.jumpSpeed = 14.5; d.maxTilt = 28.0; d.lookTilt = 25.0;
            d.mass = 0.85; d.biped = false; d.bound = 0.40; d.boundPitch = 7.0;
            d.zone = 0.95; d.swingMin = 0.12; d.swingMax = 0.38; d.size = 1.0;
            defs[3] = d;
        }

        // 4: BASTION
        {
            MechDef d = new MechDef();
            d.name = "BASTION";
            d.kindId = 4;
            d.faction = 1;
            d.blurb = "Armored sagittal octopod - heavy tank with under-hull legs";
            d.plane = GaitCore.PLANE_SAGITTAL;
            d.coxa = 0.0;
            d.nSegs = 2;
            d.lengths[0] = 1.0; d.lengths[1] = 1.1;
            d.sides[0] = -1;
            d.widths[0] = 0.34; d.widths[1] = 0.28;
            d.limits.hasRoll = true; d.limits.roll = 45.0;
            d.limits.hasHip = true; d.limits.hip[0] = -85.0; d.limits.hip[1] = 30.0;
            d.limits.bendCount = 1; d.limits.bend[0][0] = 10.0; d.limits.bend[0][1] = 150.0;
            d.addPair(-1.35, -0.45, 2.0, -1.75, 2.15);
            d.addPair(-1.35, -0.45, 0.7, -1.75, 0.75);
            d.addPair(-1.35, -0.45, -0.7, -1.75, -0.75);
            d.addPair(-1.35, -0.45, -2.0, -1.75, -2.05);
            d.bodyHalf[0] = 1.45; d.bodyHalf[1] = 0.7; d.bodyHalf[2] = 2.6;
            d.ride = 1.75;
            d.speedSlow = 9.8; d.speedWalk = 25.5; d.speedRun = 33.3;
            d.accel = 3.0; d.accelRun = 9.0; d.turnRate = 1.8;
            d.gaitCount = Gait.gaitsOctopod(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "wave");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "metachronal");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "tetrapod_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "tetrapod");
            d.stepHeight = 0.4; d.stepMax = 0.65; d.mantleMax = 1.4; d.wireMax = 0.0;
            d.climber = false; d.jumpSpeed = 10.0; d.maxTilt = 14.0; d.lookTilt = 25.0;
            d.mass = 4.0; d.biped = false; d.bound = 0.30; d.boundPitch = 4.0;
            d.zone = 0.8; d.swingMin = 0.16; d.swingMax = 0.45; d.size = 1.0;
            defs[4] = d;
        }

        // 5: WARDEN
        {
            MechDef d = new MechDef();
            d.name = "WARDEN";
            d.kindId = 5;
            d.faction = 1;
            d.blurb = "Artillery octopod - 3-segment radial legs, heavy turret";
            d.plane = GaitCore.PLANE_RADIAL;
            d.coxa = 0.3;
            d.nSegs = 3;
            d.lengths[0] = 1.8; d.lengths[1] = 1.7; d.lengths[2] = 0.6;
            d.sides[0] = 1; d.sides[1] = 1;
            d.widths[0] = 0.3; d.widths[1] = 0.24; d.widths[2] = 0.16;
            d.limits.hasCoxa = true; d.limits.coxa = 70.0;
            d.limits.hasHip = true; d.limits.hip[0] = -20.0; d.limits.hip[1] = 65.0;
            d.limits.bendCount = 2;
            d.limits.bend[0][0] = 35.0; d.limits.bend[0][1] = 150.0;
            d.limits.bend[1][0] = 15.0; d.limits.bend[1][1] = 145.0;
            d.addPair(-1.2, 0.0, 1.5, -2.7, 2.6);
            d.addPair(-1.2, 0.0, 0.5, -3.0, 0.9);
            d.addPair(-1.2, 0.0, -0.5, -3.0, -0.9);
            d.addPair(-1.2, 0.0, -1.5, -2.7, -2.5);
            d.bodyHalf[0] = 1.3; d.bodyHalf[1] = 0.65; d.bodyHalf[2] = 2.2;
            d.ride = 1.4;
            d.speedSlow = 9.8; d.speedWalk = 25.5; d.speedRun = 33.3;
            d.accel = 4.5; d.accelRun = 13.5; d.turnRate = 2.0;
            d.gaitCount = Gait.gaitsOctopod(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "wave");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "metachronal");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "tetrapod_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "tetrapod");
            d.stepHeight = 0.45; d.stepMax = 0.75; d.mantleMax = 1.8; d.wireMax = 0.0;
            d.climber = false; d.jumpSpeed = 11.0; d.maxTilt = 18.0; d.lookTilt = 25.0;
            d.mass = 4.0; d.biped = false; d.bound = 0.35; d.boundPitch = 5.0;
            d.zone = 0.95; d.swingMin = 0.14; d.swingMax = 0.45; d.size = 1.0;
            defs[5] = d;
        }

        // 6: MANTIS
        {
            MechDef d = new MechDef();
            d.name = "MANTIS";
            d.kindId = 6;
            d.faction = 1;
            d.blurb = "Hunter hexapod - agile 3-seg flanker with foreleg blades";
            d.plane = GaitCore.PLANE_RADIAL;
            d.coxa = 0.2;
            d.nSegs = 3;
            d.lengths[0] = 1.0; d.lengths[1] = 1.8; d.lengths[2] = 0.7;
            d.sides[0] = 1; d.sides[1] = 1;
            d.widths[0] = 0.18; d.widths[1] = 0.2; d.widths[2] = 0.12;
            d.limits.hasCoxa = true; d.limits.coxa = 75.0;
            d.limits.hasHip = true; d.limits.hip[0] = -25.0; d.limits.hip[1] = 100.0;
            d.limits.bendCount = 2;
            d.limits.bend[0][0] = 10.0; d.limits.bend[0][1] = 160.0;
            d.limits.bend[1][0] = 10.0; d.limits.bend[1][1] = 155.0;
            d.addPair(-0.55, -0.35, 0.8, -1.9, 2.0);
            d.addPair(-0.55, -0.35, 0.0, -2.1, 0.1);
            d.addPair(-0.55, -0.35, -0.8, -1.9, -1.8);
            d.bodyHalf[0] = 0.6; d.bodyHalf[1] = 0.45; d.bodyHalf[2] = 1.5;
            d.ride = 1.45;
            d.speedSlow = 14.0; d.speedWalk = 36.4; d.speedRun = 47.6;
            d.accel = 12.0; d.accelRun = 36.0; d.turnRate = 4.4;
            d.gaitCount = Gait.gaitsHexapod(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "wave");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "ripple");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "tripod_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "tripod");
            d.stepHeight = 0.5; d.stepMax = 0.75; d.mantleMax = 1.8; d.wireMax = 0.0;
            d.climber = false; d.jumpSpeed = 16.7; d.maxTilt = 24.0; d.lookTilt = 25.0;
            d.mass = 1.0; d.biped = false; d.bound = 0.55; d.boundPitch = 9.0;
            d.zone = 1.0; d.swingMin = 0.11; d.swingMax = 0.38; d.size = 1.0;
            defs[6] = d;
        }

        // 7: SCARAB
        {
            MechDef d = new MechDef();
            d.name = "SCARAB";
            d.kindId = 7;
            d.faction = 1;
            d.blurb = "Utility scout hexapod - compact frame, sensor head";
            d.plane = GaitCore.PLANE_RADIAL;
            d.coxa = 0.2;
            d.nSegs = 2;
            d.lengths[0] = 1.2; d.lengths[1] = 1.45;
            d.sides[0] = 1;
            d.widths[0] = 0.17; d.widths[1] = 0.12;
            d.limits.hasCoxa = true; d.limits.coxa = 75.0;
            d.limits.hasHip = true; d.limits.hip[0] = -25.0; d.limits.hip[1] = 75.0;
            d.limits.bendCount = 1; d.limits.bend[0][0] = 40.0; d.limits.bend[0][1] = 165.0;
            d.addPair(-0.5, -0.1, 0.6, -1.55, 1.5);
            d.addPair(-0.5, -0.1, 0.0, -1.75, 0.0);
            d.addPair(-0.5, -0.1, -0.6, -1.55, -1.4);
            d.bodyHalf[0] = 0.55; d.bodyHalf[1] = 0.38; d.bodyHalf[2] = 0.95;
            d.ride = 0.95;
            d.speedSlow = 11.2; d.speedWalk = 29.1; d.speedRun = 38.1;
            d.accel = 10.0; d.accelRun = 30.0; d.turnRate = 4.0;
            d.gaitCount = Gait.gaitsHexapod(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "wave");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "ripple");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "tripod_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "tripod");
            d.stepHeight = 0.36; d.stepMax = 0.6; d.mantleMax = 1.2; d.wireMax = 0.0;
            d.climber = false; d.jumpSpeed = 13.4; d.maxTilt = 26.0; d.lookTilt = 25.0;
            d.mass = 0.9; d.biped = false; d.bound = 0.30; d.boundPitch = 7.0;
            d.zone = 0.75; d.swingMin = 0.11; d.swingMax = 0.38; d.size = 1.0;
            defs[7] = d;
        }

        // 8: COLOSSUS
        {
            MechDef d = new MechDef();
            d.name = "COLOSSUS";
            d.kindId = 8;
            d.faction = 1;
            d.blurb = "Siege octopod - super-heavy walker with dual cannons";
            d.plane = GaitCore.PLANE_RADIAL;
            d.coxa = 0.45;
            d.nSegs = 3;
            d.lengths[0] = 2.6; d.lengths[1] = 2.6; d.lengths[2] = 0.9;
            d.sides[0] = 1; d.sides[1] = 1;
            d.widths[0] = 0.45; d.widths[1] = 0.36; d.widths[2] = 0.24;
            d.limits.hasCoxa = true; d.limits.coxa = 65.0;
            d.limits.hasHip = true; d.limits.hip[0] = -15.0; d.limits.hip[1] = 55.0;
            d.limits.bendCount = 2;
            d.limits.bend[0][0] = 40.0; d.limits.bend[0][1] = 150.0;
            d.limits.bend[1][0] = 15.0; d.limits.bend[1][1] = 145.0;
            d.addPair(-1.85, -0.1, 2.3, -4.0, 3.7);
            d.addPair(-1.85, -0.1, 0.8, -4.4, 1.3);
            d.addPair(-1.85, -0.1, -0.8, -4.4, -1.3);
            d.addPair(-1.85, -0.1, -2.3, -4.0, -3.5);
            d.bodyHalf[0] = 2.0; d.bodyHalf[1] = 1.0; d.bodyHalf[2] = 3.2;
            d.ride = 2.3;
            d.speedSlow = 7.0; d.speedWalk = 18.2; d.speedRun = 23.8;
            d.accel = 2.5; d.accelRun = 7.5; d.turnRate = 1.2;
            d.gaitCount = Gait.gaitsOctopod(d.gaits);
            d.gaitSlowIdx = Gait.find(d.gaits, d.gaitCount, "wave");
            d.gaitWalkIdx = Gait.find(d.gaits, d.gaitCount, "metachronal");
            d.gaitRunIdx = Gait.find(d.gaits, d.gaitCount, "bound");
            d.gaitWalkFastIdx = Gait.find(d.gaits, d.gaitCount, "tetrapod_run");
            d.gaitDriftIdx = Gait.find(d.gaits, d.gaitCount, "tetrapod");
            d.stepHeight = 0.7; d.stepMax = 1.2; d.mantleMax = 2.6; d.wireMax = 0.0;
            d.climber = false; d.jumpSpeed = 8.9; d.maxTilt = 12.0; d.lookTilt = 25.0;
            d.mass = 6.0; d.biped = false; d.bound = 0.30; d.boundPitch = 4.0;
            d.zone = 1.5; d.swingMin = 0.22; d.swingMax = 0.65; d.size = 1.6;
            defs[8] = d;
        }

        return defs;
    }

    static MechDef byKind(int kind) {
        int k = kind % BUILTIN_COUNT;
        if (k < 0) {
            k += BUILTIN_COUNT;
        }
        return BUILTINS[k];
    }

    /** Validates and converts a host {@link GcMechDesc} into an internal {@link MechDef}, or null if invalid. */
    static MechDef fromDesc(GcMechDesc desc, int kindId) {
        if (desc == null) {
            return null;
        }
        if (desc.numLegs != 2 && desc.numLegs != 4 && desc.numLegs != 6 && desc.numLegs != 8) {
            return null;
        }
        if (desc.numSegments < 2 || desc.numSegments > GaitCore.MAX_SEGS) {
            return null;
        }
        if (!Float.isFinite(desc.coxa) || desc.coxa < 0.0f) {
            return null;
        }
        if (!Float.isFinite(desc.rideHeight) || desc.rideHeight <= 0.05f) {
            return null;
        }
        for (int k = 0; k < 3; k++) {
            if (!Float.isFinite(desc.bodyHalf[k]) || desc.bodyHalf[k] <= 0.05f) {
                return null;
            }
        }
        for (int s = 0; s < desc.numSegments; s++) {
            if (!Float.isFinite(desc.segmentLengths[s]) || desc.segmentLengths[s] <= 0.01f) {
                return null;
            }
        }
        for (int i = 0; i < desc.numLegs; i++) {
            for (int k = 0; k < 3; k++) {
                if (!Float.isFinite(desc.legs[i].mount[k])) {
                    return null;
                }
            }
            for (int k = 0; k < 2; k++) {
                if (!Float.isFinite(desc.legs[i].restXz[k])) {
                    return null;
                }
            }
        }

        MechDef out = new MechDef();
        out.name = (desc.name != null && !desc.name.isEmpty()) ? desc.name : "CUSTOM";
        out.blurb = desc.blurb != null ? desc.blurb : "";
        out.kindId = kindId;
        out.faction = desc.faction;
        out.plane = (desc.planeMode == GaitCore.LEG_PLANE_SAGITTAL) ? GaitCore.PLANE_SAGITTAL : GaitCore.PLANE_RADIAL;
        out.coxa = desc.coxa;
        out.nSegs = desc.numSegments;
        for (int s = 0; s < out.nSegs; s++) {
            out.lengths[s] = desc.segmentLengths[s];
            out.widths[s] = desc.segmentWidths[s] > 0.0f ? desc.segmentWidths[s] : 0.20;
        }
        for (int s = 0; s < out.nSegs - 1; s++) {
            out.sides[s] = desc.bendSides[s] < 0 ? -1 : 1;
        }

        out.limits.hasCoxa = desc.limits.hasCoxa != 0;
        out.limits.coxa = desc.limits.coxaDeg;
        out.limits.hasRoll = desc.limits.hasRoll != 0;
        out.limits.roll = desc.limits.rollDeg;
        out.limits.hasHip = desc.limits.hasHip != 0;
        out.limits.hip[0] = desc.limits.hipDeg[0];
        out.limits.hip[1] = desc.limits.hipDeg[1];
        out.limits.bendCount = Math.max(0, Math.min(GaitCore.MAX_SEGS - 1, desc.limits.bendCount));
        for (int s = 0; s < out.limits.bendCount; s++) {
            out.limits.bend[s][0] = desc.limits.bendDeg[s][0];
            out.limits.bend[s][1] = desc.limits.bendDeg[s][1];
        }

        out.nLegs = desc.numLegs;
        for (int i = 0; i < out.nLegs; i++) {
            LegDef dst = out.legs[i];
            GcMechDesc.LegDesc src = desc.legs[i];
            dst.mount[0] = src.mount[0];
            dst.mount[1] = src.mount[1];
            dst.mount[2] = src.mount[2];
            dst.rest[0] = src.restXz[0];
            dst.rest[1] = src.restXz[1];
            dst.side = src.side != 0 ? (src.side < 0 ? -1 : 1)
                    : ((src.mount[0] < 0.0f || (src.mount[0] == 0.0f && src.restXz[0] < 0.0f)) ? -1 : 1);
        }

        for (int k = 0; k < 3; k++) {
            out.bodyHalf[k] = desc.bodyHalf[k];
        }
        out.ride = desc.rideHeight;
        out.speedSlow = desc.speedSlow > 0.0f ? desc.speedSlow : 8.0;
        out.speedWalk = desc.speedWalk > 0.0f ? desc.speedWalk : 20.0;
        out.speedRun = desc.speedRun > 0.0f ? desc.speedRun : 30.0;
        out.accel = desc.accel > 0.0f ? desc.accel : 8.0;
        out.accelRun = desc.accelRun > 0.0f ? desc.accelRun : 24.0;
        out.turnRate = desc.turnRate > 0.0f ? desc.turnRate : 3.2;
        out.stepHeight = desc.stepHeight > 0.0f ? desc.stepHeight : 0.45;
        out.stepMax = desc.stepMax > 0.0f ? desc.stepMax : 0.7;
        out.mantleMax = desc.mantleMax > 0.0f ? desc.mantleMax : 1.6;
        out.wireMax = desc.wireMax > 0.0f ? desc.wireMax : 0.0;
        out.climber = desc.climber != 0;
        out.jumpSpeed = desc.jumpSpeed > 0.0f ? desc.jumpSpeed : 12.0;
        out.maxTilt = desc.maxTiltDeg > 0.0f ? desc.maxTiltDeg : 18.0;
        out.lookTilt = desc.lookTiltDeg > 0.0f ? desc.lookTiltDeg : 25.0;
        out.mass = desc.mass > 0.0f ? desc.mass : 1.0;
        out.bound = desc.bound > 0.0f ? desc.bound : 0.0;
        out.boundPitch = desc.boundPitchDeg;
        out.zone = desc.zone > 0.0f ? desc.zone : 1.0;
        out.swingMin = desc.swingMin > 0.0f ? desc.swingMin : 0.12;
        out.swingMax = desc.swingMax > out.swingMin ? desc.swingMax : (out.swingMin + 0.25);
        out.size = desc.size > 0.0f ? desc.size : 1.0;

        if (out.nLegs == 2) {
            out.biped = true;
            out.gaitCount = Gait.gaitsBiped(out.gaits);
            out.gaitSlowIdx = Gait.find(out.gaits, out.gaitCount, "stalk");
            out.gaitWalkIdx = Gait.find(out.gaits, out.gaitCount, "walk");
            out.gaitRunIdx = Gait.find(out.gaits, out.gaitCount, "run");
            out.gaitWalkFastIdx = -1;
            out.gaitDriftIdx = out.gaitWalkIdx;
        } else if (out.nLegs == 4) {
            out.biped = false;
            out.gaitCount = Gait.gaitsQuadruped(out.gaits);
            out.gaitSlowIdx = Gait.find(out.gaits, out.gaitCount, "crawl");
            out.gaitWalkIdx = Gait.find(out.gaits, out.gaitCount, "walk");
            out.gaitRunIdx = Gait.find(out.gaits, out.gaitCount, "bound");
            out.gaitWalkFastIdx = Gait.find(out.gaits, out.gaitCount, "trot_run");
            out.gaitDriftIdx = Gait.find(out.gaits, out.gaitCount, "trot");
        } else if (out.nLegs == 6) {
            out.biped = false;
            out.gaitCount = Gait.gaitsHexapod(out.gaits);
            out.gaitSlowIdx = Gait.find(out.gaits, out.gaitCount, "wave");
            out.gaitWalkIdx = Gait.find(out.gaits, out.gaitCount, "ripple");
            out.gaitRunIdx = Gait.find(out.gaits, out.gaitCount, "bound");
            out.gaitWalkFastIdx = Gait.find(out.gaits, out.gaitCount, "tripod_run");
            out.gaitDriftIdx = Gait.find(out.gaits, out.gaitCount, "tripod");
        } else {
            out.biped = false;
            out.gaitCount = Gait.gaitsOctopod(out.gaits);
            out.gaitSlowIdx = Gait.find(out.gaits, out.gaitCount, "wave");
            out.gaitWalkIdx = Gait.find(out.gaits, out.gaitCount, "metachronal");
            out.gaitRunIdx = Gait.find(out.gaits, out.gaitCount, "bound");
            out.gaitWalkFastIdx = Gait.find(out.gaits, out.gaitCount, "tetrapod_run");
            out.gaitDriftIdx = Gait.find(out.gaits, out.gaitCount, "tetrapod");
        }
        return out;
    }

    boolean sameTemplate(MechDef o) {
        if (o == null) return false;
        if (!name.equals(o.name) || !blurb.equals(o.blurb) || faction != o.faction
                || plane != o.plane || Double.compare(coxa, o.coxa) != 0 || nSegs != o.nSegs) {
            return false;
        }
        if (!Arrays.equals(lengths, o.lengths) || !Arrays.equals(sides, o.sides)
                || !Arrays.equals(widths, o.widths)) {
            return false;
        }
        if (limits.hasCoxa != o.limits.hasCoxa || Double.compare(limits.coxa, o.limits.coxa) != 0
                || limits.hasRoll != o.limits.hasRoll || Double.compare(limits.roll, o.limits.roll) != 0
                || limits.hasHip != o.limits.hasHip
                || !Arrays.equals(limits.hip, o.limits.hip)
                || limits.bendCount != o.limits.bendCount
                || !Arrays.deepEquals(limits.bend, o.limits.bend)) {
            return false;
        }
        if (nLegs != o.nLegs) return false;
        for (int i = 0; i < nLegs; i++) {
            LegDef a = legs[i];
            LegDef b = o.legs[i];
            if (!Arrays.equals(a.mount, b.mount) || !Arrays.equals(a.rest, b.rest)
                    || a.side != b.side || a.hasSides != b.hasSides
                    || !Arrays.equals(a.sides, b.sides) || a.hasLimits != b.hasLimits) {
                return false;
            }
        }
        return Arrays.equals(bodyHalf, o.bodyHalf)
                && Double.compare(ride, o.ride) == 0
                && Double.compare(speedSlow, o.speedSlow) == 0
                && Double.compare(speedWalk, o.speedWalk) == 0
                && Double.compare(speedRun, o.speedRun) == 0
                && Double.compare(accel, o.accel) == 0
                && Double.compare(accelRun, o.accelRun) == 0
                && Double.compare(turnRate, o.turnRate) == 0
                && Double.compare(stepHeight, o.stepHeight) == 0
                && Double.compare(stepMax, o.stepMax) == 0
                && Double.compare(mantleMax, o.mantleMax) == 0
                && Double.compare(wireMax, o.wireMax) == 0
                && climber == o.climber
                && Double.compare(jumpSpeed, o.jumpSpeed) == 0
                && Double.compare(maxTilt, o.maxTilt) == 0
                && Double.compare(lookTilt, o.lookTilt) == 0
                && Double.compare(mass, o.mass) == 0
                && biped == o.biped
                && Double.compare(bound, o.bound) == 0
                && Double.compare(boundPitch, o.boundPitch) == 0
                && Double.compare(zone, o.zone) == 0
                && Double.compare(swingMin, o.swingMin) == 0
                && Double.compare(swingMax, o.swingMax) == 0
                && Double.compare(size, o.size) == 0;
    }

    static boolean descFromPreset(int presetKind, GcMechDesc outDesc) {
        if (outDesc == null || presetKind < 0 || presetKind >= BUILTIN_COUNT) {
            return false;
        }
        MechDef d = byKind(presetKind);
        outDesc.clear();
        outDesc.name = d.name;
        outDesc.blurb = d.blurb;
        outDesc.faction = d.faction;
        outDesc.planeMode = d.plane == GaitCore.PLANE_SAGITTAL ? GaitCore.LEG_PLANE_SAGITTAL : GaitCore.LEG_PLANE_RADIAL;
        outDesc.coxa = (float) d.coxa;
        outDesc.numSegments = d.nSegs;
        for (int s = 0; s < d.nSegs && s < GaitCore.MAX_SEGS_PUBLIC; s++) {
            outDesc.segmentLengths[s] = (float) d.lengths[s];
            outDesc.segmentWidths[s] = (float) d.widths[s];
        }
        for (int s = 0; s < d.nSegs - 1 && s < GaitCore.MAX_SEGS_PUBLIC - 1; s++) {
            outDesc.bendSides[s] = d.sides[s];
        }
        outDesc.limits.hasCoxa = d.limits.hasCoxa ? 1 : 0;
        outDesc.limits.coxaDeg = (float) d.limits.coxa;
        outDesc.limits.hasRoll = d.limits.hasRoll ? 1 : 0;
        outDesc.limits.rollDeg = (float) d.limits.roll;
        outDesc.limits.hasHip = d.limits.hasHip ? 1 : 0;
        outDesc.limits.hipDeg[0] = (float) d.limits.hip[0];
        outDesc.limits.hipDeg[1] = (float) d.limits.hip[1];
        outDesc.limits.bendCount = d.limits.bendCount;
        for (int s = 0; s < d.limits.bendCount && s < GaitCore.MAX_SEGS_PUBLIC - 1; s++) {
            outDesc.limits.bendDeg[s][0] = (float) d.limits.bend[s][0];
            outDesc.limits.bendDeg[s][1] = (float) d.limits.bend[s][1];
        }

        outDesc.numLegs = d.nLegs;
        for (int i = 0; i < d.nLegs && i < GaitCore.MAX_LEGS_PUBLIC; i++) {
            outDesc.legs[i].mount[0] = (float) d.legs[i].mount[0];
            outDesc.legs[i].mount[1] = (float) d.legs[i].mount[1];
            outDesc.legs[i].mount[2] = (float) d.legs[i].mount[2];
            outDesc.legs[i].restXz[0] = (float) d.legs[i].rest[0];
            outDesc.legs[i].restXz[1] = (float) d.legs[i].rest[1];
            outDesc.legs[i].side = d.legs[i].side;
        }
        for (int k = 0; k < 3; k++) {
            outDesc.bodyHalf[k] = (float) d.bodyHalf[k];
        }
        outDesc.rideHeight = (float) d.ride;
        outDesc.speedSlow = (float) d.speedSlow;
        outDesc.speedWalk = (float) d.speedWalk;
        outDesc.speedRun = (float) d.speedRun;
        outDesc.accel = (float) d.accel;
        outDesc.accelRun = (float) d.accelRun;
        outDesc.turnRate = (float) d.turnRate;
        outDesc.stepHeight = (float) d.stepHeight;
        outDesc.stepMax = (float) d.stepMax;
        outDesc.mantleMax = (float) d.mantleMax;
        outDesc.wireMax = (float) d.wireMax;
        outDesc.climber = d.climber ? 1 : 0;
        outDesc.jumpSpeed = (float) d.jumpSpeed;
        outDesc.maxTiltDeg = (float) d.maxTilt;
        outDesc.lookTiltDeg = (float) d.lookTilt;
        outDesc.mass = (float) d.mass;
        outDesc.bound = (float) d.bound;
        outDesc.boundPitchDeg = (float) d.boundPitch;
        outDesc.zone = (float) d.zone;
        outDesc.swingMin = (float) d.swingMin;
        outDesc.swingMax = (float) d.swingMax;
        outDesc.size = (float) d.size;
        return true;
    }
}
