package titans.roads;

import org.apache.commons.math3.util.FastMath;
import org.javatuples.Pair;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import javax.annotation.Nonnull;
import java.util.ArrayList;
import java.util.function.BiFunction;

/**
 * This class represents a trapezoidal motion profile that plans velocity and acceleration for a {@link Road}.
 * The class builds the whole profile by itself and requires a {@link ConstraintSet} and a Kinematics object (WIP).
 *
 * <p>
 *     !todo: write about sampling and resolution
 * </p>
 * <p>
 *     After building, the object is immutable, and it is intended for the user to pass the profile into a
 *     {@link InterpolationLUT} to get planned velocity and acceleration values for all points on a trajectory.
 *     The computation of the profile is very computationally-expensive depending on the resolution. A higher resolution
 *     gives a better profile, while decreasing performance.
 * </p>
 *
 * @author AlexD70
 * @see InterpolationLUT
 * @see Road
 * @see ConstraintSet
 */
public class TrapezoidalMotionProfile {
    Road road;
    double duration;
    int resolution;
    HolonomicKinematics kin;

    private final ArrayList<KinematicState> timeProfile = new ArrayList<>();

    /**
     * Internal ctor that builds an empty motion profile. Should not be used directly unless inheiriting from this class.
     * @param resolution the resolution of the profile. Should be at least 3.
     * @param kin the Kinematics object
     * @param r the {@code Road} to build the profile for
     */
    protected TrapezoidalMotionProfile(int resolution, @Nonnull HolonomicKinematics kin, Road r){
        if(resolution <= 2){
            throw new RuntimeException("Resolution is too low to build a motion profile");
        }

        this.resolution = resolution;
        this.kin = kin;
        this.road = r;
    }

    /**
     * This constructor builds the full motion profile from its parameters. It samples the displacement profile,
     * then passes twice through the result (forwards and backwards) and finalizes by reparameterizing to time.
     * The displacement profile is a local variable of the ctor such that at the end, only the time profile remains
     * to reduce the memory occupied by the motion profile.
     * @param road the {@link Road} to build the profile for
     * @param constraints the {@link ConstraintSet} for the robot
     * @param initVel velocity at the start of the road (absolute value)
     * @param endVel velocity at the end of the road (absolute value)
     * @param initAcc acceleration at the start of the road (absolute value)
     * @param xWheelOffset robot wheel offset on x-axis
     * @param yWheelOffset robot wheel offset on y-axis
     * @param resolution profile resolution. Should be at least 3
     */
    public TrapezoidalMotionProfile(@Nonnull Road road, @Nonnull ConstraintSet constraints, double initVel, double endVel, double initAcc, double xWheelOffset, double yWheelOffset, int resolution){
        this(resolution, new HolonomicKinematics(xWheelOffset, yWheelOffset, constraints), road);
        ArrayList<Pair<Double, Double>> dispProfile = new ArrayList<>(resolution + 1);

        int n = this.sampleDisplacementProfile(dispProfile, initVel, initAcc, endVel);

        this.forwardPass(dispProfile, n);
        this.backwardsPass(dispProfile, n);

        this.duration = this.turnToTimeProfile(dispProfile, n, initVel, initAcc);
    }

    /**
     * Generates a linear space between 2 values at a given resolution representing the number of points
     * between the start and end value. Also adds the endpoints.
     * @param start start value
     * @param end end value
     * @param res resolution
     * @return a new {@code double[res]}
     */
    private static double[] linearSpace(double start, double end, int res){
        double[] linspace = new double[res];
        double dist = end - start;

        double space = dist / (res - 1);
        linspace[0] = start;
        for(int i = 1; i < res; i++){
            linspace[i] = linspace[0] + i * space;
        }
        if(res > 1) {
            linspace[res - 1] = end;
        }

        return linspace;
    }

    /**
     * Adds a kinematic state to the **time** profile. The state is built from the parameters given.
     * @param time time coordinate
     * @param xy position
     * @param vx velocity on x-axis
     * @param vy velocity on y-axis
     * @param ax acceleration on x-axis
     * @param ay acceleration on y-axis
     * @see KinematicState
     */
    private void addToProfile(double time, @Nonnull Point2d xy, double vx, double vy, double ax, double ay){
        timeProfile.add(new KinematicState(
                time, xy, new Vector2d(vx, vy), new Vector2d(ax, ay)
        ));
    }

    /**
     * This function samples the displacement profile {@code this.dispProfile} by adding points on as many displacement coordinates as
     * the resolution of the profile indicates. The added points will be kinematic states comprised of the max
     * planned velocity of the robot in absolute value. The maximum velocity varies depending on the robot kinematics,
     * the kinematic constraints and the path curvature.
     * @param dispProfile the profile to pass through
     * @param initVel initial velocity, usually 0
     * @param initAcc initial acceleration, usually 0
     * @param endVel end velocity, usually 0
     * @return the number of states in the sampled profile
     * @throws RuntimeException if the curvature is {@code null} at any point
     */
    private int sampleDisplacementProfile(@Nonnull ArrayList<Pair<Double, Double>> dispProfile, double initVel, double initAcc, double endVel){
        double[] linspaceDisplacement = linearSpace(0, road.getLength(), this.resolution);

        int lastSgm = 0;
        Vector2d prevTan = null;
        // add the start
        dispProfile.add(0, new Pair<>(0d, initVel));

        for(int i = 1; i < linspaceDisplacement.length; i++){
            //double angVelLimitedByCurvature = max_ang_vel / FastMath.abs(curvature * 2);
            /*if (Double.isNaN(angVelLimitedByCurvature) || Double.isInfinite(angVelLimitedByCurvature) || curvature < 0.01){
                angVelLimitedByCurvature = max_vel + 1;
            }*/

            double d = linspaceDisplacement[i];
            dispProfile.add(i, new Pair<>(d, kin.getVelocityLimit(d, road, prevTan, lastSgm)));

            Point2d pointDeriv = road.getDerivAtDisp(d);
            if(pointDeriv == null){
                prevTan = null;
            } else {
                prevTan = pointDeriv.toVector().norm();
            }

            lastSgm = road.getSegment1Disp(d);
        }

        // add the end to the displacement profile
        dispProfile.add(this.resolution - 1, new Pair<>(road.getLength(), endVel));

        return linspaceDisplacement.length - 1;
    }

    private double rungeKutta4(double step, double tn, @Nonnull BiFunction<Double, Double, Double> f, double yn){
        double k1 = f.apply(tn, yn);
        double k2 = f.apply(tn + step/2, yn + k1 * step/2);
        double k3 = f.apply(tn + step/2, yn + k2 * step/2);
        double k4 = f.apply(tn + step, yn + step * k3);

        return yn + step/6d * (k1 + 2 * k2 + 2 * k3 + k4);
    }

    /**
     * Passes forward through the displacement profile {@code this.dispProfile} to ensure the robot
     * can achieve the desired speed with respect to its maximum acceleration.
     * @param dispProfile the profile to pass through
     * @param n the number of kinematic states in the displacement profile
     */
    private void forwardPass(@Nonnull ArrayList<Pair<Double, Double>> dispProfile, int n){
        for (int j = 1; j <= n; j ++){
            Pair<Double, Double> current = dispProfile.get(j);
            Pair<Double, Double> prev = dispProfile.get(j - 1);

            double currentVel = current.getValue1(), prevVel = prev.getValue1();
            if(currentVel > prevVel){
                double d1 = current.getValue0(), d2 = prev.getValue0();
                double deltaDisp = d1 - d2;

                double accGuess = kin.getLinearAcceleration(prevVel, road.getCurvatureAtDisp(d1));
                double velGuess = FastMath.sqrt(prevVel * prevVel + 2 * deltaDisp * accGuess);
                velGuess = FastMath.min(velGuess, currentVel);
                double availableLinearAcc = kin.getLinearAcceleration(velGuess, road.getCurvatureAtDisp(d1));

                /*double maxVelRK4 = rungeKutta4(deltaDisp, d1,
                        (d, v) -> 2 * kin.getLinearAcceleration(v, road.getCurvatureAtDisp(d)), prevVel
                );*/

                double maxVel = FastMath.sqrt(FastMath.pow(prevVel, 2) + 2 * deltaDisp * availableLinearAcc);
                dispProfile.set(j, new Pair<>(d1, FastMath.min(maxVel, currentVel)));
            }
        }
    }

    /**
     * Passes backwards through the displacement profile {@code this.dispProfile} to ensure the robot
     * can achieve the desired speed with respect to its maximum deceleration.
     * @param dispProfile the profile to pass through
     * @param n the number of kinematic states in the displacement profile
     */
    private void backwardsPass(@Nonnull ArrayList<Pair<Double, Double>> dispProfile, int n){
        // backwards pass
        for (int j = n; j > 0; j--){
            Pair<Double, Double> current = dispProfile.get(j);
            Pair<Double, Double> next = dispProfile.get(j - 1);

            double currentVel = current.getValue1(), nextVel = next.getValue1();
            if(currentVel < nextVel){
                double d1 = current.getValue0(), d2 = next.getValue0();
                double deltaDisp = d1 - d2;

                double accGuess = kin.getLinearAcceleration(currentVel, road.getCurvatureAtDisp(d2));
                double velGuess = FastMath.sqrt(FastMath.pow(currentVel, 2) + 2 * deltaDisp * accGuess);
                velGuess = FastMath.min(velGuess, nextVel);
                double availableLinearAcc = kin.getLinearAcceleration(velGuess, road.getCurvatureAtDisp(d2));

                /*double maxVelRK4 = rungeKutta4(deltaDisp, d2,
                        (d, v) -> 2 * kin.getLinearAcceleration(v, road.getCurvatureAtDisp(d)), currentVel
                );*/

                double maxVel = FastMath.sqrt(FastMath.pow(currentVel, 2) + 2 * deltaDisp * availableLinearAcc);
                dispProfile.set(j - 1, new Pair<>(d2, FastMath.min(maxVel, nextVel)));
            }
        }
    }

    /**
     * Reparameterizes the displacement profile for time.
     * @param dispProfile the profile to reparameterize
     * @param n the number of kinematic states in the profile
     * @param initVel initial velocity
     * @param initAcc initial acceleration
     * @return the total time of the profile
     */
    private double turnToTimeProfile(@Nonnull ArrayList<Pair<Double, Double>> dispProfile, int n, double initVel, double initAcc){
        // turn to time based profile
        this.timeProfile.ensureCapacity(n);

        Vector2d tanVec = road.getDerivAtDispNonnull(0).toVector().norm();

        this.addToProfile(
                0d,
                road.pointAtDisp(0),
                initVel * tanVec.getX(),
                initVel * tanVec.getY(), // project the velocity and acc on the tangent vector
                initAcc * tanVec.getX(),
                initAcc * tanVec.getY()
        );


        double lastTime = 0, time = 0;
        double lastVel, currentVel;
        for (int j = 1; j <= n; j ++){
            Pair<Double, Double> current = dispProfile.get(j), last = dispProfile.get(j - 1);

            currentVel = current.getValue1();
            lastVel = last.getValue1();
            double ds = current.getValue0() - last.getValue0();

            double dt = 0;
            if(FastMath.abs(currentVel + lastVel) < 1e-6){
                dt = FastMath.sqrt(2 * ds / this.kin.maxAcc());
            } else {
                dt = FastMath.abs(ds / ((currentVel + lastVel) / 2)); // ds / med_vel
            }

            time += dt;

            tanVec = road.getDerivAtDispNonnull(current.getValue0()).toVector().norm(); // normalize the vector

            double vx = currentVel * tanVec.getX(), vy = currentVel * tanVec.getY();

            //double accel = (currentVel - lastVel) / dt;
            Double curv = road.getCurvatureAtDisp(current.getValue0());
            double centripetalAccel = 0;
            if (curv != null) {
                centripetalAccel = FastMath.pow(currentVel, 2) * curv;
            }

            // a = dv/dt = d(v^2)/(2*ds)
            double accel = 0;
            if (j < n) {
                Pair<Double, Double> next = dispProfile.get(j + 1);
                double ds_central = next.getValue0() - last.getValue0();
                double dv2_central = FastMath.pow(next.getValue1(), 2) - FastMath.pow(lastVel, 2);
                accel = dv2_central / (2.0 * ds_central);
            } else {
                // Fallback backward difference for the very last point
                accel = (FastMath.pow(currentVel, 2) - FastMath.pow(lastVel, 2)) / (2.0 * ds);
            }

            double maxAccTan = kin.getLinearAcceleration(currentVel, curv);
            accel = FastMath.max(-maxAccTan, FastMath.min(accel, maxAccTan));

            // Tangential component
            double ax = accel * tanVec.getX();
            double ay = accel * tanVec.getY();

            // Add Normal (Centripetal) component: perpendicular to tangent
            ax += centripetalAccel * (-tanVec.getY());
            ay += centripetalAccel * tanVec.getX();

            Point2d point = road.pointAtDisp(current.getValue0());
            this.addToProfile(
                    time,
                    point,
                    vx, vy, ax, ay
            );

            lastTime = time;
        }

        return time;
    }

    public int getProfileSize(){
        return timeProfile.size();
    }

    public @Nonnull KinematicState getStateByIdx(int index){
        return timeProfile.get(index);
    }

    @Override
    public String toString() {
        StringBuilder s = new StringBuilder(super.toString() + "\n\t");
        for(KinematicState state : timeProfile){
            if(state == null){
                s.append("NULL!!!\n\t");
            } else {
                s.append(state).append("\n\t");
            }
        }

        return s.toString();
    }
}
