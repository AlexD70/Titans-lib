package titans.roads;

public class HolonomicKinematics {
    private double xWheelOffset = 0, yWheelOffset = 0;
    private ConstraintSetXY constraints;

    public HolonomicKinematics(ConstraintSetXY constr){
        constraints = constr;
    }

    public HolonomicKinematics(double x, double y, ConstraintSetXY constr){
        xWheelOffset = x;
        yWheelOffset = y;
        constraints = constr;
    }

    public void setWheelOffsets(double x, double y){
        xWheelOffset = x;
        yWheelOffset = y;
    }

    public double maxVel() {
        return constraints.maxVel;
    }

    public double maxAcc() {
        return constraints.maxAccel;
    }

    public double maxAngVel() {
        return constraints.maxAngVel;
    }

    public double maxAngAcc() {
        return constraints.maxAngAccel;
    }

    public double calculateTranslationalScale(double vx, double vy, double omega) {
        return 0;
    }
}
