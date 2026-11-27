import gaitcore.GaitCore;
import gaitcore.GcContext;
import gaitcore.GcMechDesc;
import gaitcore.GcMechInfo;
import gaitcore.GcMechStats;
import gaitcore.GcStubWorld;

/**
 * Minimal standalone Java game-engine integration example for GaitCore.
 *
 * <p>Shows how to:
 * <ol>
 *   <li>Create a thread-safe world geometry callback (using {@link GcStubWorld} here as the stand-in).</li>
 *   <li>Define a custom multi-legged mech from scratch via {@link GcMechDesc}.</li>
 *   <li>Register / spawn it into a {@link GcContext} alongside a customized built-in preset.</li>
 *   <li>Run a 60 Hz simulation loop and read back packed body/joint poses, events, and stats.</li>
 * </ol>
 */
public final class CustomMechExample {

    private CustomMechExample() {
    }

    /** Builds a custom medium quadruped mech from scratch. */
    private static GcMechDesc makeCustomQuadruped() {
        GcMechDesc d = GcMechDesc.createDefault(4);
        d.name = "MY_QUAD_SCOUT";
        d.blurb = "Custom 4-legged scout defined in user game code";
        d.planeMode = GaitCore.LEG_PLANE_RADIAL;

        // Leg segment proportions (coxa + thigh + shin). Keep rest reach ~55-60% of max reach.
        d.coxa = 0.30f;
        d.numSegments = 2;
        d.segmentLengths[0] = 1.65f;
        d.segmentLengths[1] = 1.85f;
        d.bendSides[0] = 1; // knee bends outward/upward
        d.segmentWidths[0] = 0.16f;
        d.segmentWidths[1] = 0.11f;

        // Joint limits in degrees.
        d.limits.hasCoxa = 1;
        d.limits.coxaDeg = 55.0f;
        d.limits.hasHip = 1;
        d.limits.hipDeg[0] = -40.0f;
        d.limits.hipDeg[1] = 85.0f;
        d.limits.bendCount = 1;
        d.limits.bendDeg[0][0] = 20.0f;
        d.limits.bendDeg[0][1] = 148.0f;

        // Body dimensions & ride height (meters).
        d.bodyHalf[0] = 0.85f;
        d.bodyHalf[1] = 0.38f;
        d.bodyHalf[2] = 1.25f;
        d.rideHeight = 1.55f;

        // Locomotion speeds & agility (m/s, m/s^2, rad/s).
        d.speedSlow = 5.0f;
        d.speedWalk = 12.0f;
        d.speedRun = 21.0f;
        d.accel = 32.0f;
        d.accelRun = 24.0f;
        d.turnRate = 3.2f;
        d.stepHeight = 0.80f;
        d.stepMax = 1.15f;
        d.mantleMax = 2.0f;
        d.wireMax = 4.0f;
        d.climber = 1;
        d.jumpSpeed = 13.0f;
        d.maxTiltDeg = 22.0f;
        d.mass = 1.2f;
        d.zone = 0.35f;
        d.swingMin = 0.16f;
        d.swingMax = 0.36f;

        // Legs listed front-to-back in left/right pairs: L0, R0, L1, R1.
        float[][] mounts = {
                { -0.70f, -0.05f,  0.90f },
                {  0.70f, -0.05f,  0.90f },
                { -0.75f, -0.05f, -0.90f },
                {  0.75f, -0.05f, -0.90f },
        };
        float[][] rests = {
                { -1.85f,  1.45f },
                {  1.85f,  1.45f },
                { -1.90f, -1.45f },
                {  1.90f, -1.45f },
        };
        for (int i = 0; i < 4; i++) {
            d.legs[i].mount[0] = mounts[i][0];
            d.legs[i].mount[1] = mounts[i][1];
            d.legs[i].mount[2] = mounts[i][2];
            d.legs[i].restXz[0] = rests[i][0];
            d.legs[i].restXz[1] = rests[i][1];
        }
        return d;
    }

    public static void main(String[] args) {
        // 1. Set up a simple world with flat ground + two step boxes ahead.
        GcStubWorld world = new GcStubWorld(0.0, true);
        world.addBox(-5.0, 0.0, 6.0, 5.0, 0.45, 11.0);
        world.addBox(-5.0, 0.0, 14.0, 5.0, 0.90, 20.0);

        // 2. Create the GaitCore runtime context (0 = auto worker thread count).
        try (GcContext ctx = GcContext.create(0)) {
            ctx.setWorld(world);

            // 3a. Register and spawn our custom quadruped archetype.
            GcMechDesc scoutDesc = makeCustomQuadruped();
            int quadKind = ctx.registerMechType(scoutDesc);
            int quadId = ctx.spawn(quadKind, -1.5f, 0.0f, 0.0f, 0.0f);

            // 3b. Spawn a tweaked biped preset directly via spawnCustom().
            GcMechDesc bipedDesc = new GcMechDesc();
            GcMechDesc.fromPreset(0, bipedDesc); // start from preset 0 ("LYNX")
            bipedDesc.name = "MY_BIPED";
            bipedDesc.speedRun *= 1.15f;
            int bipedId = ctx.spawnCustom(bipedDesc, 2.0f, 0.0f, 0.0f, 0.0f);

            GcMechInfo info = new GcMechInfo();
            ctx.info(quadId, info);
            System.out.printf("Spawned %s (kind=%d, legs=%d, joints/leg=%d, floats/mech=%d)%n",
                    info.name, quadKind, info.legs, info.jointsPerLeg, info.floats);

            // 4. Allocate output buffers big enough for all spawned mechs.
            int totalFloats = ctx.floatsPerMech(quadId) + ctx.floatsPerMech(bipedId);
            float[] poses = new float[totalFloats];
            float[] inputs = new float[2 * GaitCore.INPUT_STRIDE];
            float[] events = new float[64 * GaitCore.EVENT_FLOATS];

            // 5. Simulate 180 frames at 60 Hz (3.0 seconds), running across the step boxes.
            GcMechStats stats = new GcMechStats();
            for (int frame = 0; frame < 180; frame++) {
                // Command mech 0 (quad) and mech 1 (biped) to run forward; quick-slide mid-run.
                inputs[0 * GaitCore.INPUT_STRIDE + GaitCore.IN_FWD] = 1.0f;
                inputs[0 * GaitCore.INPUT_STRIDE + GaitCore.IN_RUN] = 1.0f;
                inputs[0 * GaitCore.INPUT_STRIDE + GaitCore.IN_DRIFT] =
                        (frame >= 90 && frame < 108) ? 1.0f : 0.0f;

                inputs[1 * GaitCore.INPUT_STRIDE + GaitCore.IN_FWD] = 1.0f;
                inputs[1 * GaitCore.INPUT_STRIDE + GaitCore.IN_RUN] = 1.0f;

                ctx.step(1.0f / 60.0f, inputs, 2);
                ctx.poses(poses, poses.length);
                int nevFloats = ctx.events(events, events.length);
                int nev = nevFloats / GaitCore.EVENT_FLOATS;

                if (frame % 30 == 29) {
                    ctx.stats(quadId, stats);
                    System.out.printf(
                            "frame=%3d  pos=(%.2f, %.2f, %.2f)  speed=%.1f m/s  gait=%-8s  events=%d%n",
                            frame + 1,
                            poses[GaitCore.POSE_POS + 0],
                            poses[GaitCore.POSE_POS + 1],
                            poses[GaitCore.POSE_POS + 2],
                            poses[GaitCore.POSE_SPEED],
                            stats.gait,
                            nev);
                }
            }
        }
    }
}
