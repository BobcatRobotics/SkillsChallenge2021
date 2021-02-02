package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    /**
     * Limelight's camera states
     */
    public enum CAM {
        VISION,
        DRIVER
    }

    private boolean initialized = false;
    private NetworkTableEntry tv = null;
    private NetworkTableEntry tx = null;
    private NetworkTableEntry ty = null;
    private NetworkTableEntry ta = null;
    private NetworkTableEntry ta0 = null;
    private NetworkTableEntry ts0 = null;
    private NetworkTableEntry ta1 = null;
    private NetworkTableEntry ts1 = null;
    /**
     * Limelight's LED states
     */
    public enum LED {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    private NetworkTable limelight;

    /**
     * Initialize the Limelight file
     */
    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelight.getEntry("tv");

        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        ta0 = limelight.getEntry("ta0");
        ts0 = limelight.getEntry("ts0");
        ta1 = limelight.getEntry("ta1");
        ts1 = limelight.getEntry("ts1");   
    }

    /**
     * @param entry The entry's ID
     * @return The network table entry
     */
    public NetworkTableEntry getEntry(String entry) {
        return limelight.getEntry(entry);
    }

    /**
     * @param entry The entry's ID
     * @return The double stored in the entry
     */
    public double getDouble(String entry) {
        return getEntry(entry).getDouble(0.0);
    }

    /**
     * @param entry The entry's ID
     * @return The boolean stored in the entry
     */
    public boolean getBoolean(String entry) {
        return getDouble(entry) == 1;
    }

    /**
     * @param entry The entry's ID
     * @return The integer stored in the entry
     */
    public int getInt(String entry) {
        return (int) getDouble(entry);
    }

    /**
     * @return Whether or not the limelight has any valid targets
     */
    public boolean hasTarget() {
        return getBoolean("tv");
    }

    public double getTargetDistance() {
        double rVal = -1.0;
        if (hasTarget()) {
            double angle = y() + 18;
            rVal = ((47.5)/12) / Math.tan(Math.toRadians(angle));
        }
        return rVal;
    }
    
    /**
     * @return Horizontal Offset From Crosshair To Target (-27 to 27 degrees)
     */
    public double x() {
        return getDouble("tx");
    }

    /**
     * @return Vertical Offset From Crosshair To Target (-20.5 to 20.5 degrees)
     */
    public double y() {
        return getDouble("ty");
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double targetArea() {
        return getDouble("ta");
    }

    /**
     * @param entryName The entry's ID
     * @param entryValue Value that the entry will be set to
     */
    public void setEntry(String entryName, int entryValue) {
        getEntry(entryName).setNumber(entryValue);
    }

    /**
     * @param state what to set the limelight LED mode to
     */
    public void setLED(int state) {
        setEntry("ledMode", state);
    }

    /**
     * @return Get the state of the LEDs
     */
    public LED getLED() {
        return LED.values()[getInt("ledMode")];
    }

    /**
     * @param state what to set the limelight camera mode to
     */
    public void setCAM(int state) {
        setEntry("camMode", state);
    }

    /**
     * @return the state of the Limelight camera
     */
    public CAM getCAM() {
        return CAM.values()[getInt("ledMode")];
    }
}