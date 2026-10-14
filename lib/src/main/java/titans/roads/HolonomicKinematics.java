package titans.roads;

public class HolonomicKinematics {
    private double xWheelOffset = 0, yWheelOffset = 0;
    private ConstraintSet constraints;

    public HolonomicKinematics(ConstraintSet constr){
        constraints = constr;
    }

    public HolonomicKinematics(double x, double y, ConstraintSet constr){
        xWheelOffset = x;
        yWheelOffset = y;
        constraints = constr;
    }

    public void setWheelOffsets(double x, double y){
        xWheelOffset = x;
        yWheelOffset = y;
    }

    public double maxVel() {
        return constraints.getMaxVel();
    }

    public double maxAcc() {
        return constraints.getMaxAcc();
    }

    public double maxAngVel() {
        return constraints.getMaxAngVel();
    }

    public double maxAngAcc() {
        return constraints.getMaxAngAcc();
    }

    public double calculateTranslationalScale(double vx, double vy, double omega) {
        return 0;
    }
}
