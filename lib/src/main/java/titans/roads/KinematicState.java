package titans.roads;

import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
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
        double interpolationFactor = FastMath.abs((actualTime - first.getTime()) / (second.getTime() - first.getTime()));

        Vector2d poseSum = first.getPos().getVector()
                .mlt(1 - interpolationFactor)
                .add(second.getPos().getVector().mlt(interpolationFactor));
        Vector2d velocitySum = first.getVel()
                .mlt(1 - interpolationFactor)
                .add(second.getVel().mlt(interpolationFactor));
        Vector2d accelerationSum = first.getAcc()
                .mlt(1 - interpolationFactor)
                .add(second.getAcc().mlt(interpolationFactor));


        double angleInterp = first.getPos().getTheta() * (1 - interpolationFactor) + second.getPos().getTheta() * interpolationFactor;
        double angularVelInterp = first.angularVelocity * (1 - interpolationFactor) + second.angularVelocity * interpolationFactor;
        double angularAccInterp = first.angularAcceleration * (1 - interpolationFactor) + second.angularAcceleration * interpolationFactor;

        angleInterp = MathUtils.normalizeAngle(angleInterp, 0d);
        angularVelInterp = MathUtils.normalizeAngle(angularVelInterp, 0d);
        angularAccInterp = MathUtils.normalizeAngle(angularAccInterp, 0d);
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

    @Override
    public String toString() {
        return super.toString() + String.format(
                " : {t = %f, pose = %s, vel = %s, acc = %s, omega = %f, eps = %f}",
                time,
                (position == null)?("NULL"):(position.toString()),
                velocity,
                acceleration,
                angularVelocity,
                angularAcceleration
        );
    }
}
