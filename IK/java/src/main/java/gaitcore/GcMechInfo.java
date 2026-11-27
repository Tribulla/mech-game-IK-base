package gaitcore;

/** Static dimension and configuration readout populated by {@link GcContext#info}. */
public final class GcMechInfo {
    public String name = "";
    public String blurb = "";
    public int kind;
    public int legs;
    public int jointsPerLeg;
    public final float[] half = new float[3];
    public final float[] widths = new float[4];
    public int widthCount;
    public float mass;
    public int climber;
    public int faction;
    public float wireMax;
    public float walkSpeed;
    public float runSpeed;
    public int floats;
}
