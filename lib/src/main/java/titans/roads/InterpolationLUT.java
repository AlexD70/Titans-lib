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
    private int maxIter = Integer.MAX_VALUE;

    public double getTime(int index){
        return getState(index).getTime();
    }

    public KinematicState getState(int index){
        return profile.getStateByIdx(index);
    }

    public KinematicState getStateAt(double t) {
        int left = 0;
        int right = profile.getProfileSize() - 1;

        // Check bounds
        if (t <= getTime(left) + eps) return getState(left);
        if (t >= getTime(right) - eps) return getState(right);

        // Linear / Step-wise search if starting near predictable indices,
        // or binary search for arbitrary lookups:
        while (left <= right) {
            int mid = left + (right - left) / 2;
            double midTime = getTime(mid);

            if (FastMath.abs(t - midTime) <= eps) {
                return getState(mid);
            }

            if (midTime < t) {
                left = mid + 1;
            } else {
                right = mid - 1;
            }
        }

        // `right` is now the index of the point just before `t`, and `right + 1` is just after `t`
        int idx1 = right;
        int idx2 = right + 1;

        double t1 = getTime(idx1);
        double t2 = getTime(idx2);

        KinematicState state1 = getState(idx1);
        KinematicState state2 = getState(idx2);

        // If getLinearInterpolation requires timestamp t:
        return KinematicState.getLinearInterpolation(state1, state2, t);

        // OR if getLinearInterpolation requires alpha (0.0 to 1.0):
        // double alpha = (t - t1) / (t2 - t1);
        // return KinematicState.getLinearInterpolation(state1, state2, alpha);
    }

    public KinematicState getStateAt2(double t){
        double time1 = this.getTime(first);
        double time2 = this.getTime(second);

        int iter = 0;
        while(iter < maxIter) {
            if (FastMath.abs(t - time1) <= eps) {
                // do not go to next here
                return getState(first);

            } else if (FastMath.abs(t - time2) <= eps) {
                first += 1;
                second += 1;
                return getState(second - 1);

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
