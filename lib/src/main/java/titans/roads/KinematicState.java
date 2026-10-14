package titans.roads;

import org.apache.commons.math3.util.FastMath;
import titans.geometry.Point2d;
import titans.geometry.Pose;
import titans.geometry.Vector2d;

/**
 * Immutable container class for kinematic states. Stores the robot {@code Pose}, meaning its 2D position and
 * orientation, its velocity and acceleration on both the x-axis and the y-axis, the time and the robot's angular
 * velocity and acceleration.
 *
 * @author AlexD70
 * @see Pose
 */
public class KinematicState {
    private final double time;
    private final Pose position;
    private final Vector2d velocity;
    private final Vector2d acceleration;
    private final double angularVelocity, angularAcceleration;

    /**
     * Builds a "time-less" kinematic state containing only position, velocity and acceleration.
     * @param pos the position
     * @param vel velocity on both x and y axes
     * @param acc acceleration on both x and y axes
     */
    public KinematicState(Point2d pos, Vector2d vel, Vector2d acc){
        position = new Pose(pos, 0);
        velocity = vel;
        acceleration = acc;
        time = 0;
        angularVelocity = 0;
        angularAcceleration = 0;
    }

    /**
     * Builds a kinematic state containing time, position, velocity and acceleration.
     * @param t the time
     * @param pos the position
     * @param vel velocity on both x and y axes
     * @param acc acceleration on both x and y axes
     */
    public KinematicState(double t, Point2d pos, Vector2d vel, Vector2d acc){
        position = new Pose(pos, 0);
        velocity = vel;
        acceleration = acc;
        time = t;
        angularVelocity = 0;
        angularAcceleration = 0;
    }

    /**
     * Builds a complete kinematic profile.
     * @param t the time
     * @param pos the position
     * @param vel velocity on both x and y axes
     * @param acc acceleration on both x and y axes
     * @param omega angular velocity
     * @param epsilon angular acceleration
     */
    public KinematicState(double t, Pose pos, Vector2d vel, Vector2d acc, double omega, double epsilon){
        position = pos;
        velocity = vel;
        acceleration = acc;
        time = t;
        angularVelocity = omega;
        angularAcceleration = epsilon;
    }

    public double getTime(){
        return time;
    }

    public Pose getPos(){
        return position;
    }

    public Vector2d getVel(){
        return velocity;
    }

    public Vector2d getAcc(){
        return acceleration;
    }

    public static KinematicState getLinearInterpolation(KinematicState first, KinematicState second, double actualTime){
        Vector2d poseSum = first.getPos().getVector().add(second.getPos().getVector());
        Vector2d velocitySum = first.getVel().add(second.getVel());
        Vector2d accelerationSum = first.getAcc().add(second.getAcc());

        double interpolationFactor = FastMath.abs((actualTime - first.getTime()) / (second.getTime() - first.getTime()));

        double angleInterp = (first.getPos().getTheta() + second.getPos().getTheta()) * interpolationFactor;
        double angularVelInterp = (first.angularVelocity + second.angularVelocity) * interpolationFactor;
        double angularAccInterp = (first.angularAcceleration + second.angularAcceleration) * interpolationFactor;

        poseSum.mlt(interpolationFactor);
        velocitySum.mlt(interpolationFactor);
        accelerationSum.mlt(interpolationFactor);

        return new KinematicState(actualTime, poseSum.toPoint().addAngle(angleInterp), velocitySum, accelerationSum, angularVelInterp, angularAccInterp);
    }

    private static final double EPS = 0.001;
    @Override
    public boolean equals(Object obj){
        if (obj instanceof KinematicState kst){
            return (
                    kst.position.equals(this.position) &&
                    kst.velocity.equals(this.velocity) &&
                    kst.acceleration.equals(this.acceleration) &&
                    kst.time == this.time &&
                    FastMath.abs(kst.angularVelocity - this.angularVelocity) < EPS &&
                    FastMath.abs(kst.angularAcceleration - this.angularAcceleration) < EPS
            );
        } else {
            return false;
        }
    }
}
