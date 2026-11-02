import org.apache.commons.math3.util.FastMath;
import org.javatuples.Pair;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.roads.*;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class ConstraintTests {
    private final double EPS = 1e-2;

    @Tag("test") @Test
    void checkProfileConstraints(){
        ConstraintSet constr = TestResources.getRandomConstraints();
        double maxv = constr.getMaxVel();
        double maxa = constr.getMaxAcc();
        double maxangv = constr.getMaxAngVel();

        Pair<TrapezoidalMotionProfile, Road> res = TestResources.getRandomProfile(constr, 2500, 10, 10);
        TrapezoidalMotionProfile prof = res.getValue0();
        Road r = res.getValue1();
        int n = prof.getProfileSize();
        double disp = 0;

        for(int i = 0; i < n; i++){
            KinematicState ret = prof.getStateByIdx(i);

            Vector2d v_actual = ret.getVel();
            assertTrue(v_actual.abs() < maxv * 1.02 + EPS, String.format("Value %f does not repect the constraint of %f", v_actual.abs(), maxv + EPS));

            Vector2d a_actual = ret.getAcc();
            assertTrue(a_actual.abs() < maxa * 1.02 + EPS, String.format("Value %f does not repect the constraint of %f", a_actual.abs(), maxa + EPS));

            Double curv = r.getCurvatureAtDisp(disp);
            if(curv != null){
                assertTrue(v_actual.abs() < maxangv / Math.abs(curv * 2) * 1.02 + EPS, String.format("Value %f does not repect the constraint of %f", v_actual.abs(), maxangv / Math.abs(curv * 2) + EPS));
            }
        }
    }

    // checks for sudden jumps
    @Tag("test") @Test
    void checkProfileSanity(){
        ConstraintSet constr = TestResources.getRandomConstraints();
        double maxv = constr.getMaxVel();
        double maxa = constr.getMaxAcc();
        double maxangv = constr.getMaxAngVel();

        Pair<TrapezoidalMotionProfile, Road> res = TestResources.getRandomProfile(constr, 2500, 10, 10);
        TrapezoidalMotionProfile prof = res.getValue0();
        Road r = res.getValue1();
        int n = prof.getProfileSize();
        double disp = 0;

        for(int i = 0; i < n - 1; i++){
            KinematicState ret1 = prof.getStateByIdx(i);
            KinematicState ret2 = prof.getStateByIdx(i + 1);

            double dt = ret2.getTime() - ret1.getTime();

            Point2d pt1 = ret1.getPos().getPoint();
            Point2d pt2 = ret2.getPos().getPoint();
            Vector2d v = pt2.diff(pt1).mlt(1/dt);
            assertTrue(v.abs() < maxv * 1.02 + EPS, String.format("Velocity %f does not repect the constraint of %f at \n%s \n%s", v.abs(), maxv + EPS, ret1, ret2));

            Vector2d v1 = ret1.getVel();
            Vector2d v2 = ret2.getVel();
            Vector2d a = v2.diff(v1).mlt(1/dt);
            assertTrue(a.abs() < maxa * 1.02 + EPS, String.format("Accel %f does not repect the constraint of %f at \n%s \n%s", a.abs(), maxa + EPS, ret1, ret2));

            Double curv = r.getCurvatureAtDisp(disp);
            if(curv != null){
                assertTrue(v.abs() < maxangv / Math.abs(curv * 2) * 1.02 + EPS, String.format("Velocity %f does not repect the (curvature) constraint of %f at \n%s \n%s", v.abs(), maxangv / Math.abs(curv * 2) + EPS, ret1, ret2));
            }
        }
    }

    @Tag("test") @Test
    void checkInterpLUTConstraints(){
        ConstraintSet constr = TestResources.getRandomConstraints();
        Pair<TrapezoidalMotionProfile, Road> res = TestResources.getRandomProfile(constr, 2500, 10, 10);

        TrapezoidalMotionProfile prof = res.getValue0();
        Road r = res.getValue1();
        InterpolationLUT lut = new InterpolationLUT(prof);

        double maxv = constr.getMaxVel();
        double maxa = constr.getMaxAcc();
        double maxangv = constr.getMaxAngVel();

        double maxTime = lut.getEndTime();
        for(int i = 0; i < 2000; i++){
            double randTime = FastMath.random() * maxTime;
            if(randTime < 0){
                randTime = 0;
            } else if (randTime > maxTime){
                randTime = maxTime;
            }

            KinematicState ret = lut.getStateAt(randTime);
            Vector2d v_actual = ret.getVel();
            assertTrue(v_actual.abs() < maxv * 1.02 + EPS, String.format("Value %f does not repect the constraint of %f", v_actual.abs(), maxv + EPS));

            Vector2d a_actual = ret.getAcc();
            assertTrue(a_actual.abs() < maxa * 1.02 + EPS, String.format("Value %f does not repect the constraint of %f", a_actual.abs(), maxa + EPS));

            //Double curv = r.getCurvatureAtDisp();
            //if(curv != null){
            //    assertTrue(v_actual.abs() < maxangv / Math.abs(curv * 2) + EPS, String.format("Value %f does not repect the constraint of %f", v_actual.abs(), maxangv / Math.abs(curv * 2) + EPS));
            //}
        }
    }
}
