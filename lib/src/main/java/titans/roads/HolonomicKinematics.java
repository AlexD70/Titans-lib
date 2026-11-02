package titans.roads;

import org.apache.commons.math3.util.FastMath;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

public class HolonomicKinematics implements RobotKinematics {
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

    @Override
    public double getVelocityLimit(double displacement, Road road, @Nullable Vector2d prevTan, int lastSegment) {
        double plannedVel = maxVel();
        Double curvature = road.getCurvatureAtDisp(displacement);
        if(curvature == null){
            throw new RuntimeException("Curvature is null??? Fatal error.");
        }

        Point2d roadDeriv = road.getDerivAtDisp(displacement);
        double tanVecAbs = 0d;
        if(roadDeriv == null){
            tanVecAbs = Double.POSITIVE_INFINITY;
        } else {
            tanVecAbs = roadDeriv.toVector().abs();
        }

        int currentSegment = road.getSegment1Disp(displacement);
        if(currentSegment != lastSegment){
            if(curvature.isInfinite() || Double.isInfinite(tanVecAbs) || !road.isContinous.get(currentSegment)){
                plannedVel = 0; // discontinuity
            }
        }

        double maxCentripetalVel = Double.POSITIVE_INFINITY;
        if (FastMath.abs(curvature) > 1e-6) { // safe float check
            maxCentripetalVel = FastMath.sqrt(maxAcc() / FastMath.abs(curvature));
        }

        Vector2d currTan = road.getDerivAtDispNonnull(displacement).toVector().norm();
        if(prevTan != null && currTan.dot(prevTan) < 0.5){
            plannedVel = 0;
        }

        return FastMath.min(plannedVel, maxCentripetalVel);
    }

    @Override
    public double getLinearAcceleration(double currentVel, @Nullable Double curvature) {
        double centripetalAcc = 0;
        if (curvature != null) {
            centripetalAcc = FastMath.pow(currentVel, 2) * FastMath.abs(curvature);
        }

        double availableLinearAcc = 0;
        if(maxAcc() > centripetalAcc){
            availableLinearAcc = FastMath.sqrt(FastMath.pow(maxAcc(), 2) - FastMath.pow(centripetalAcc, 2));
        }

        return availableLinearAcc;
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
