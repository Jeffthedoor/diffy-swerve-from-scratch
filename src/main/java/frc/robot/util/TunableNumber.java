package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber extends Number{
    private final String tableKey = "TunableNumbers";

    private String key;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;

    private DoubleSubscriber subscriber;

    /**
     * Create a new TunableNumber
     * 
     * @param name Key on dashboard
     */
    private TunableNumber(String name) {
        this.key = tableKey + "/" + name;
    }

    /**
     * Create a new TunableNumber with the default value
     * 
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunableNumber(String name, double defaultValue) {
        this(name);
        setDefault(defaultValue);
    }

    /**
     * Create a new TunableNumber with the default value
     * 
     * @param name Key on dashboard
     * @param defaultValue Default value
     */
    public TunableNumber(String name, double defaultValue, String tableKey) {
        this(name);
        this.key = tableKey;
        setDefault(defaultValue);
    }

    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    private void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuningMode) {
            subscriber = NetworkTableInstance.getDefault().getTable(tableKey).getDoubleTopic(key).subscribe(defaultValue);
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     * 
     * @return The current value
     */
    private double get() {
        return Constants.tuningMode ? subscriber.get()
                : defaultValue;
    }

    /**
     * Checks whether the number has changed since our last check
     * 
     * @return True if the number has changed since the last time this method was called, false
     *                 otherwise
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }

        return false;
    }

    @Override
    public int intValue() {
        return (int) get();
    }
    @Override
    public long longValue() {
        return (long) get();
    }
    @Override
    public float floatValue() {
        return (float) get();
    }
    @Override
    public double doubleValue() {
        return get();
    }
    @Override
    public String toString() {
        return String.valueOf(get());
    }
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof TunableNumber) {
            return ((TunableNumber) obj).get() == get();
        }
        return false;
    }
    @Override
    public int hashCode() {
        return Double.hashCode(get());
    }
}