package gaitcore;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Public Java entry point for the GaitCore simulation context.
 *
 * <p>Mirrors {@code GcContext} in {@code gaitcore.h}: each spawned mech is stepped independently
 * across a persistent {@link ThreadPool}, then events and telemetry are drained sequentially in
 * {@code 0..count()-1} handle order so simulation output is bit-identical across thread counts.
 */
public final class GcContext implements AutoCloseable {

    private ThreadPool pool;
    private GcWorldCallbacks worldCb;
    private final List<Mech> mechs = new ArrayList<>();
    private final List<MechDef> customDefs = new ArrayList<>();

    private float[] eventsBuf = new float[0];
    private int eventFloats;

    private float[] dbgRaysBuf = new float[0];
    private int dbgRayFloats;

    private final GcBenchConfig bench = new GcBenchConfig();
    private boolean debug;
    private final float[] profBuf = new float[GaitCore.PROFILE_FLOATS];
    private final GcQueryStats queryStats = new GcQueryStats();
    private float lastWallMs;
    private float lastCpuMs;
    private int lastRays;

    private int[] poseOffsets = new int[0];

    private GcContext(int numThreads) {
        this.pool = new ThreadPool(numThreads);
    }

    /** Creates a context and persistent worker pool. Pass {@code <= 0} to auto-detect cores. */
    public static GcContext create(int numThreads) {
        return new GcContext(numThreads);
    }

    @Override
    public void close() {
        clearCustomTypes();
        mechs.clear();
        if (pool != null) {
            pool.close();
            pool = null;
        }
    }

    private MechDef defByKind(int kind) {
        int baseN = MechDef.BUILTIN_COUNT;
        if (kind >= 0 && kind < baseN) {
            return MechDef.byKind(kind);
        }
        int ci = kind - baseN;
        if (ci >= 0 && ci < customDefs.size()) {
            return customDefs.get(ci);
        }
        return null;
    }

    private void applyBenchCfg(Mech m, int idx) {
        m.ikSolveHz = bench.legIkHz;
        m.probePeriod = 1.0 / GaitCore.clamp(bench.probeHz, 1.0, 120.0);
        m.speedScale = GaitCore.clamp(bench.speedScale, 0.0, 3.0);
        m.planeRollEnabled = bench.planeRoll != 0;
        m.profileEnabled = bench.profile != 0;
        m.ikAcc = (m.ikSolveHz > 0.0) ? ((idx % 8) * (0.125 / m.ikSolveHz)) : 0.0;
        m.wq.record = debug;
        m.wq.profileQueries = bench.profile != 0;
        m.wq.cb = worldCb;
    }

    private void resetDefaultBenchConfig() {
        bench.maxSubstep = (float) GaitCore.MAX_SUBSTEP_DEFAULT;
        bench.legIkHz = 0.0f;
        bench.probeHz = 20.0f;
        bench.speedScale = 1.0f;
        bench.planeRoll = 1;
        bench.profile = 0;
    }

    /** Despawns all active mechs and clears transient stats while retaining custom archetypes. */
    public void reset() {
        mechs.clear();
        eventFloats = 0;
        dbgRayFloats = 0;
        debug = false;
        resetDefaultBenchConfig();
        Arrays.fill(profBuf, 0.0f);
        resetQueryStats();
        lastWallMs = 0.0f;
        lastCpuMs = 0.0f;
        lastRays = 0;
    }

    /** Installs the host world query callbacks. Must stay read-only and thread-safe during {@link #step}. */
    public void setWorld(GcWorldCallbacks world) {
        worldCb = world;
        for (int i = 0; i < mechs.size(); i++) {
            mechs.get(i).wq.cb = worldCb;
        }
    }

    /** Resizes the worker pool (call between frames, never concurrently with {@link #step}). */
    public void setThreads(int numThreads) {
        if (numThreads <= 0) {
            numThreads = GaitCore.hardwareThreads();
        }
        numThreads = Math.max(1, Math.min(64, numThreads));
        if (pool != null && pool.threadCount() == numThreads) {
            return;
        }
        if (pool != null) {
            pool.close();
        }
        pool = new ThreadPool(numThreads);
    }

    public int getThreads() {
        return pool != null ? pool.threadCount() : 1;
    }

    /** Validates and registers a custom mech archetype, returning kind ID or -1 on error. */
    public int registerMechType(GcMechDesc desc) {
        if (desc == null) {
            return -1;
        }
        int kindId = MechDef.BUILTIN_COUNT + customDefs.size();
        MechDef def = MechDef.fromDesc(desc, kindId);
        if (def == null) {
            return -1;
        }
        customDefs.add(def);
        return kindId;
    }

    public int contextKindCount() {
        return MechDef.BUILTIN_COUNT + customDefs.size();
    }

    public String contextKindName(int kind) {
        MechDef d = defByKind(kind);
        return d != null ? d.name : "?";
    }

    /** Despawns all mechs and frees custom archetypes registered in this context. */
    public void clearCustomTypes() {
        despawnTo(0);
        customDefs.clear();
    }

    /** Spawns a mech at ground point {@code (x, y, z)} facing {@code yaw} radians. Returns handle or -1. */
    public int spawn(int kind, float x, float y, float z, float yaw) {
        MechDef d = defByKind(kind);
        if (d == null) {
            return -1;
        }
        Mech m = new Mech(d, new Vec3(x, y, z), yaw, worldCb);
        int idx = mechs.size();
        applyBenchCfg(m, idx);
        mechs.add(m);
        return idx;
    }

    /** Registers (or reuses an identical matching descriptor) and spawns one instance. */
    public int spawnCustom(GcMechDesc desc, float x, float y, float z, float yaw) {
        if (desc == null) {
            return -1;
        }
        MechDef tmp = MechDef.fromDesc(desc, 0);
        if (tmp == null) {
            return -1;
        }
        for (int i = 0; i < customDefs.size(); i++) {
            if (customDefs.get(i).sameTemplate(tmp)) {
                return spawn(MechDef.BUILTIN_COUNT + i, x, y, z, yaw);
            }
        }
        int kind = registerMechType(desc);
        if (kind < 0) {
            return -1;
        }
        return spawn(kind, x, y, z, yaw);
    }

    public int respawn(int handle, int kind, float x, float y, float z, float yaw) {
        if (handle < 0 || handle >= mechs.size()) {
            return -1;
        }
        MechDef d = defByKind(kind);
        if (d == null) {
            return -1;
        }
        Mech m = new Mech(d, new Vec3(x, y, z), yaw, worldCb);
        applyBenchCfg(m, handle);
        mechs.set(handle, m);
        return handle;
    }

    public int despawnTo(int targetCount) {
        if (targetCount < 0) {
            targetCount = 0;
        }
        while (mechs.size() > targetCount) {
            mechs.remove(mechs.size() - 1);
        }
        return mechs.size();
    }

    public int count() {
        return mechs.size();
    }

    public int floatsPerMech(int handle) {
        if (handle < 0 || handle >= mechs.size()) {
            return 0;
        }
        return mechs.get(handle).floatsNeeded();
    }

    public boolean info(int handle, GcMechInfo out) {
        if (out == null || handle < 0 || handle >= mechs.size()) {
            return false;
        }
        Mech m = mechs.get(handle);
        MechDef d = m.d;
        out.name = d.name;
        out.blurb = d.blurb;
        out.kind = d.kindId;
        out.legs = m.n;
        out.jointsPerLeg = m.jointsPerLeg;
        out.half[0] = (float) d.bodyHalf[0];
        out.half[1] = (float) d.bodyHalf[1];
        out.half[2] = (float) d.bodyHalf[2];
        out.widthCount = Math.min(d.nSegs, 4);
        Arrays.fill(out.widths, 0.0f);
        for (int i = 0; i < out.widthCount; i++) {
            out.widths[i] = (float) d.widths[i];
        }
        out.mass = (float) d.mass;
        out.climber = d.climber ? 1 : 0;
        out.faction = d.faction;
        out.wireMax = (float) d.wireMax;
        out.walkSpeed = (float) d.speedWalk;
        out.runSpeed = (float) d.speedRun;
        out.floats = floatsPerMech(handle);
        return true;
    }

    public void setBenchConfig(GcBenchConfig cfg) {
        if (cfg == null) {
            return;
        }
        bench.maxSubstep = (float) GaitCore.clamp(cfg.maxSubstep, 1.0 / 240.0, GaitCore.MAX_FRAME_DT);
        bench.legIkHz = (float) GaitCore.clamp(cfg.legIkHz, 0.0, 240.0);
        bench.probeHz = (float) GaitCore.clamp(cfg.probeHz, 1.0, 120.0);
        bench.speedScale = (float) GaitCore.clamp(cfg.speedScale, 0.0, 3.0);
        bench.planeRoll = cfg.planeRoll != 0 ? 1 : 0;
        bench.profile = cfg.profile != 0 ? 1 : 0;
        for (int i = 0; i < mechs.size(); i++) {
            applyBenchCfg(mechs.get(i), i);
        }
    }

    private void appendEventsBuf(float[] src, int nFloats) {
        if (nFloats <= 0) {
            return;
        }
        if (eventFloats + nFloats > eventsBuf.length) {
            int nc = eventsBuf.length > 0 ? eventsBuf.length * 2 : 1024 * GaitCore.EVENT_FLOATS;
            while (nc < eventFloats + nFloats) {
                nc *= 2;
            }
            eventsBuf = Arrays.copyOf(eventsBuf, nc);
        }
        System.arraycopy(src, 0, eventsBuf, eventFloats, nFloats);
        eventFloats += nFloats;
    }

    private void appendDbgRaysBuf(float[] src, int nRays) {
        if (nRays <= 0) {
            return;
        }
        int need = nRays * GaitCore.DEBUG_LINE_FLOATS;
        if (dbgRayFloats + need > dbgRaysBuf.length) {
            int nc = dbgRaysBuf.length > 0 ? dbgRaysBuf.length * 2 : 1024 * GaitCore.DEBUG_LINE_FLOATS;
            while (nc < dbgRayFloats + need) {
                nc *= 2;
            }
            dbgRaysBuf = Arrays.copyOf(dbgRaysBuf, nc);
        }
        int dst = dbgRayFloats;
        for (int i = 0; i < nRays; i++) {
            int s = i * 7;
            int role = (src[s + 6] > 0.5f) ? GaitCore.DBG_WARNING : GaitCore.DBG_LINE_DIM;
            dbgRaysBuf[dst] = src[s];
            dbgRaysBuf[dst + 1] = src[s + 1];
            dbgRaysBuf[dst + 2] = src[s + 2];
            dbgRaysBuf[dst + 3] = src[s + 3];
            dbgRaysBuf[dst + 4] = src[s + 4];
            dbgRaysBuf[dst + 5] = src[s + 5];
            dbgRaysBuf[dst + 6] = (float) role;
            dbgRaysBuf[dst + 7] = 0.0f;
            dbgRaysBuf[dst + 8] = 0.0f;
            dst += GaitCore.DEBUG_LINE_FLOATS;
        }
        dbgRayFloats += need;
    }

    /** Advances all active mechs in parallel across workers ({@code dt} clamped to {@code <= 0.1 s}). */
    public void step(float dtIn, float[] inputs, int mechCount) {
        if (!Float.isFinite(dtIn) || dtIn <= 0.0f) {
            return;
        }
        double dt = Math.min((double) dtIn, GaitCore.MAX_FRAME_DT);
        int count = mechs.size();
        int nMechs = Math.min(count, mechCount);
        if (inputs != null && nMechs > 0) {
            for (int i = 0; i < nMechs; i++) {
                mechs.get(i).inp.set(inputs, i * GaitCore.INPUT_STRIDE);
            }
        }

        dbgRayFloats = 0;
        double maxSub = GaitCore.clamp(bench.maxSubstep, 1.0 / 240.0, GaitCore.MAX_FRAME_DT);
        final int substeps = Math.max(1, (int) Math.ceil(dt / maxSub - 1e-6));
        final double sub = dt / substeps;
        final boolean prof = bench.profile != 0;

        double t0 = GaitCore.timeSec();
        if (count > 0) {
            pool.parallelFor(count, idx -> {
                Mech m = mechs.get(idx);
                m.wq.clearRays();
                m.eventCount = 0;
                Arrays.fill(m.prof, 0.0);
                double mt0 = prof ? GaitCore.timeSec() : 0.0;
                for (int s = 0; s < substeps; s++) {
                    m.update(sub);
                }
                m.stepMs = prof ? ((GaitCore.timeSec() - mt0) * 1000.0) : 0.0;
            });
        }

        double totalRays = 0.0;
        double[] profSum = new double[10];
        double cpuMs = 0.0;
        int[] kindCnt = new int[GaitCore.PROFILE_KINDS];
        int[] kindLegs = new int[GaitCore.PROFILE_KINDS];
        double[] kindMs = new double[GaitCore.PROFILE_KINDS];
        int[] kindRays = new int[GaitCore.PROFILE_KINDS];
        int[] kindIk = new int[GaitCore.PROFILE_KINDS];

        for (int i = 0; i < count; i++) {
            Mech m = mechs.get(i);
            // Drain sequentially after the barrier so output matches single-thread order byte-for-byte.
            if (m.eventCount > 0) {
                appendEventsBuf(m.eventData, m.eventCount * GaitCore.EVENT_FLOATS);
            }
            if (debug && m.wq.rayFloats > 0) {
                appendDbgRaysBuf(m.wq.rays, m.wq.rayFloats / 7);
            }

            totalRays += m.wq.rayCount;
            queryStats.rayCalls += m.wq.rayCount;
            queryStats.pushCalls += m.wq.pushCount;
            queryStats.heightCalls += m.wq.heightCount;
            queryStats.rayMs += (float) m.wq.rayMs;
            queryStats.pushMs += (float) m.wq.pushMs;
            queryStats.heightMs += (float) m.wq.heightMs;

            if (prof) {
                cpuMs += m.stepMs;
                for (int s = 0; s < 10; s++) {
                    profSum[s] += m.prof[s];
                }
                int kid = m.d.kindId;
                if (0 <= kid && kid < GaitCore.PROFILE_KINDS) {
                    kindCnt[kid] += 1;
                    kindLegs[kid] += m.n;
                    kindMs[kid] += m.stepMs;
                    kindRays[kid] += m.wq.rayCount;
                    kindIk[kid] += (int) m.prof[8];
                }
            }
        }

        double wallMs = (GaitCore.timeSec() - t0) * 1000.0;
        lastWallMs = (float) wallMs;
        lastCpuMs = (float) cpuMs;
        lastRays = (int) totalRays;

        if (prof) {
            float prevPosesMs = profBuf[13];
            double stageSumMs = 0.0;
            for (int s = 0; s < 8; s++) {
                stageSumMs += profSum[s] * 1000.0;
            }
            double wallScale = (stageSumMs > wallMs && stageSumMs > 1e-6) ? (wallMs / stageSumMs) : 1.0;
            double kindScale = (cpuMs > wallMs && cpuMs > 1e-6) ? (wallMs / cpuMs) : 1.0;

            profBuf[0] = (float) wallMs;
            for (int s = 0; s < 8; s++) {
                profBuf[1 + s] = (float) (profSum[s] * 1000.0 * wallScale);
            }
            profBuf[9] = substeps;
            profBuf[10] = (float) profSum[8];
            profBuf[11] = (float) profSum[9];
            profBuf[12] = (float) totalRays;
            profBuf[13] = prevPosesMs;

            int totalLegs = 0;
            for (int k = 0; k < GaitCore.PROFILE_KINDS; k++) {
                totalLegs += kindLegs[k];
            }
            profBuf[14] = (float) totalLegs;
            profBuf[15] = (float) count;

            for (int k = 0; k < GaitCore.PROFILE_KINDS; k++) {
                int base = GaitCore.PROFILE_HEADER + k * GaitCore.PROFILE_KIND_STRIDE;
                profBuf[base] = (float) kindCnt[k];
                profBuf[base + 1] = (float) kindLegs[k];
                profBuf[base + 2] = (float) (kindMs[k] * kindScale);
                profBuf[base + 3] = (float) kindRays[k];
                profBuf[base + 4] = (float) kindIk[k];
            }
        }
    }

    /** Packs all mech poses into {@code outBuf} and returns the number of floats written. */
    public int poses(float[] outBuf, int maxFloats) {
        int count = mechs.size();
        if (outBuf == null || maxFloats <= 0 || count <= 0) {
            return 0;
        }
        int limit = Math.min(maxFloats, outBuf.length);
        boolean prof = bench.profile != 0;
        double t0 = prof ? GaitCore.timeSec() : 0.0;
        if (poseOffsets.length < count) {
            poseOffsets = new int[count];
        }
        int total = 0;
        for (int i = 0; i < count; i++) {
            poseOffsets[i] = total;
            total += mechs.get(i).floatsNeeded();
        }
        if (count >= 16 && getThreads() > 1) {
            pool.parallelFor(count, idx -> {
                int off = poseOffsets[idx];
                if (off < limit) {
                    mechs.get(idx).pack(outBuf, off, limit - off);
                }
            });
        } else {
            for (int i = 0; i < count; i++) {
                int off = poseOffsets[i];
                if (off >= limit) {
                    break;
                }
                mechs.get(i).pack(outBuf, off, limit - off);
            }
        }
        if (prof) {
            profBuf[13] = (float) ((GaitCore.timeSec() - t0) * 1000.0);
        }
        return Math.min(total, limit);
    }

    public int poses(float[] outBuf) {
        return outBuf == null ? 0 : poses(outBuf, outBuf.length);
    }

    /** Copies accumulated events since the last drain and clears the internal event buffer. */
    public int events(float[] outBuf, int maxFloats) {
        if (outBuf == null || maxFloats <= 0 || eventFloats <= 0) {
            return 0;
        }
        int n = Math.min(eventFloats, Math.min(maxFloats, outBuf.length));
        System.arraycopy(eventsBuf, 0, outBuf, 0, n);
        eventFloats = 0;
        return n;
    }

    public int debugLines(int handle, float[] outBuf, int maxFloats) {
        if (outBuf == null || maxFloats <= 0) {
            return 0;
        }
        int limit = Math.min(maxFloats, outBuf.length);
        int n = 0;
        if (handle >= 0 && handle < mechs.size()) {
            n = mechs.get(handle).debugLines(outBuf, limit);
        }
        if (debug && dbgRayFloats > 0 && n < limit) {
            int rem = limit - n;
            int copyN = Math.min(dbgRayFloats, rem);
            System.arraycopy(dbgRaysBuf, 0, outBuf, n, copyN);
            n += copyN;
        }
        return n;
    }

    public void setDebug(boolean on) {
        debug = on;
        for (int i = 0; i < mechs.size(); i++) {
            mechs.get(i).wq.record = debug;
        }
    }

    public boolean stats(int handle, GcMechStats out) {
        if (out == null || handle < 0 || handle >= mechs.size()) {
            return false;
        }
        mechs.get(handle).fillStats(lastWallMs, lastRays, out);
        return true;
    }

    public int profileData(float[] outBuf, int maxFloats) {
        if (outBuf == null || maxFloats <= 0) {
            return 0;
        }
        int n = Math.min(GaitCore.PROFILE_FLOATS, Math.min(maxFloats, outBuf.length));
        System.arraycopy(profBuf, 0, outBuf, 0, n);
        return n;
    }

    public void resetQueryStats() {
        queryStats.rayCalls = 0;
        queryStats.pushCalls = 0;
        queryStats.heightCalls = 0;
        queryStats.rayMs = 0.0f;
        queryStats.pushMs = 0.0f;
        queryStats.heightMs = 0.0f;
    }

    public GcQueryStats getQueryStats() {
        GcQueryStats copy = new GcQueryStats();
        copy.rayCalls = queryStats.rayCalls;
        copy.pushCalls = queryStats.pushCalls;
        copy.heightCalls = queryStats.heightCalls;
        copy.rayMs = queryStats.rayMs;
        copy.pushMs = queryStats.pushMs;
        copy.heightMs = queryStats.heightMs;
        return copy;
    }

    public float lastCpuMs() {
        return lastCpuMs;
    }
}
