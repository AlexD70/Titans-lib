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

    private static final double eps = 0.0001;

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

    public double getEndTime(){
        return getTime(profile.getProfileSize() - 1);
    }

    public InterpolationLUT(TrapezoidalMotionProfile profile){
        this.profile = profile;
    }

}
