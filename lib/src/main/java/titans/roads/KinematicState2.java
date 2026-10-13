package titans.roads;

import org.apache.commons.math3.util.FastMath;
import titans.geometry.Point2d;
import titans.geometry.Pose;
import titans.geometry.Vector2d;

// unimplemented
public class KinematicState2 {
    public Pose pose;
    public Vector2d velocity;
    public Vector2d acceleration;
    public double omega, epsilon;


    public KinematicState2(Pose pose, Vector2d vel, Vector2d acc, double ang_vel, double ang_acc){
        this.pose = pose;
        velocity = vel;
        acceleration = acc;
        omega = ang_vel;
        epsilon = ang_acc;
    }

    public static KinematicState2 getLinearInterpolation(KinematicState2 first, KinematicState2 second, double timetotal, double timefromfirst){
        Vector2d possum = first.pose.getVector().add(second.pose.getVector());
        Vector2d velsum = first.velocity.add(second.velocity);
        Vector2d accsum = first.acceleration.add(second.acceleration);

        double interpolation_factor = timefromfirst / timetotal;

        double anginterp = (first.pose.getTheta() + second.pose.getTheta()) * interpolation_factor;
        double angvelinterp = (first.omega + second.omega) * interpolation_factor;
        double angaccinterp = (first.epsilon + second.epsilon) * interpolation_factor;

        possum.mlt(interpolation_factor);
        velsum.mlt(interpolation_factor);
        accsum.mlt(interpolation_factor);

        return new KinematicState2(new Pose(possum.toPoint(), anginterp), velsum, accsum, angvelinterp, angaccinterp);
    }

    private static final double EPS = 0.001;
    @Override
    public boolean equals(Object obj){
        if (obj instanceof KinematicState2 kst){
            return (
                    kst.pose.equals(this.pose) &&
                    kst.velocity.equals(this.velocity) &&
                    kst.acceleration.equals(this.acceleration) &&
                    FastMath.abs(kst.omega - this.omega) < EPS &&
                    FastMath.abs(kst.epsilon - this.epsilon) < EPS
            );
        } else {
            return false;
        }
    }
}
