package gaitcore;

/** Aggregate world query call counts and accumulated callback milliseconds. */
public final class GcQueryStats {
    public int rayCalls;
    public int pushCalls;
    public int heightCalls;
    public float rayMs;
    public float pushMs;
    public float heightMs;
}
