package titans.roads;

import org.apache.commons.math3.util.FastMath;

/**
 * A lookup table for motion profiles that linearly interpolates between kinematic states.
 * Once this object is created, the road is no longer useful for trajectory
 * following purposes and can and SHOULD be deleted.
 *
 * @author AlexD70
 */
public class InterpolationLUT {
    private final TrapezoidalMotionProfile profile;

    private int first = 0, second = 1;
    private static final double eps = 0.0001;
    private int maxIter = 3;

    public double getTime(int index){
        return getState(index).getTime();
    }

    public KinematicState getState(int index){
        return profile.getStateByIdx(index);
    }

    public KinematicState getStateAt(double t){
        double time1 = this.getTime(first);
        double time2 = this.getTime(second);

        int iter = 0;
        while(iter < maxIter) {
            if (FastMath.abs(t - first) <= eps) {
                // do not go to next here
                return getState(first);

            } else if (FastMath.abs(t - second) <= eps) {
                first += 1;
                second += 1;
                return getState(second);

            } else if (time1 < t && t < time2) {
                // do not go to next in this branch
                KinematicState state1 = getState(first);
                KinematicState state2 = getState(second);

                return KinematicState.getLinearInterpolation(state1, state2, t);

            } else {
                first += 1;
                second += 1;
                iter += 1;
                time1 = getState(first).getTime();
                time2 = getState(second).getTime();
            }
        }

        // throw error if the correct value cannot be found in 4 iterations
        throw new RuntimeException("Interpolation LUT crashed at runtime. Iteration number exceeds maxIter.");
    }

    public double getEndTime(){
        return getTime(profile.getProfileSize() - 1);
    }

    // should not be used by the user
    // only for internal library use
    public void forceReset(){
        first = 0;
        second = 1;
    }

    // only for internal library use
    public void setMaxIter(int newMaxIter){
        this.maxIter = newMaxIter;
    }

    public InterpolationLUT(TrapezoidalMotionProfile profile){
        this.profile = profile;
    }

}
