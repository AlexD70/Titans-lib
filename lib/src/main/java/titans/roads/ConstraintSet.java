package titans.roads;

/**
 * Immutable container class for kinematic constraints. Stores the maximum velocity,
 * maximum acceleration, maximum angular velocity and maximum angular acceleration.
 * @author AlexD70
 */
public class ConstraintSet {
    private final double maxVel;
    private final double maxAccel;
    private final double maxAngVel;
    private final double maxAngAccel;

    /**
     * Builds the object.
     * @param maxVel the maximum velocity
     * @param maxAccel the maximum acceleration
     * @param maxAngVel the maximum angular velocity, in radians per sec
     * @param maxAngAccel the maximum angular acceleration, in radians per sec^2
     */
    public ConstraintSet(double maxVel, double maxAccel, double maxAngVel, double maxAngAccel){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxAngAccel = maxAngAccel;
        this.maxAngVel = maxAngVel;
    }

    public double getMaxVel(){
        return maxVel;
    }

    public double getMaxAcc(){
        return maxAccel;
    }

    public double getMaxAngVel(){
        return maxAngVel;
    }

    public double getMaxAngAcc(){
        return maxAngAccel;
    }
}
