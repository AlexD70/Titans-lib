import org.apache.commons.math3.util.FastMath;
import org.javatuples.Pair;
import org.javatuples.Triplet;
import titans.geometry.Point2d;
import titans.geometry.Pose;
import titans.geometry.Vector2d;
import titans.math.Spline;
import titans.roads.ConstraintSet;
import titans.roads.Road;
import titans.roads.RoadBuilder;
import titans.roads.TrapezoidalMotionProfile;

import java.awt.*;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestResources {
    public static final long MAX_MILLIS_INTEGRATION = 7, MAX_MILLIS_BRENTOPT = 15;

    // should make a separate assertions class because they are all over the place
    // same with test roads -- they're just everywhere
    // also this epsilon might be quite high -- should try testing newton raphson
    // doesnt matter - fixed precision issues with better relative tolerance and initial guess
    public static final double EPS = 0.001;
    static void assertPointsEqual(Point2d p1, Point2d p2){
        System.out.printf("(%f, %f) - expected, (%f, %f) - actual%n", p1.getX(), p1.getY(), p2.getX(), p2.getY());
        System.out.printf("(%f, %f) - delta%n", p1.getX() - p2.getX(), p1.getY() - p2.getY());
        assertWithinError(p1.getX() - p2.getX(), 0, EPS);
        assertWithinError(p1.getY() - p2.getY(), 0, EPS);
        //assertEquals(p1, p2); // Point2d.equals has a minuscule EPS value and therefore the assertion fails
    }

    public static void assertWithinError(double val, double ref){
        assertTrue(Math.abs(val - ref) <= EPS,
                String.format("Value %f out of the expected %f +- %f interval.", val, ref, EPS));
    }
    public static void assertInInterval(double val, double low, double high){
        assertTrue((low <= val) && (val <= high),
                String.format("Value %f out of interval [%f, %f]", val, low, high));
    }
    public static void assertWithinError(double val, double ref, double eps){
        assertTrue(Math.abs(val - ref) <= eps,
                String.format("Value %f out of the expected %f +- %f interval.", val, ref, eps));
    }

    public static Point2d getRandomPoint(){
        double x = FastMath.random() * 10;
        double y = FastMath.random() * 10;

        return new Point2d(x, y);
    }

    public static double getRandomAngle(){
        return FastMath.random() * FastMath.PI - FastMath.PI;
    }

    public static Spline getRandomSpline(){
        RoadBuilder builder = new RoadBuilder(getRandomPoint(), getRandomAngle(), 0, getRandomAngle());
        Road r = builder.spline(
                new Pose(getRandomPoint(), getRandomAngle()),
                getRandomAngle(),
                getRandomAngle(),
                RoadBuilder.HeadingInterpMethod.SPLINE,
                false
        ).build();

        return (Spline) r.getSegmentByIndex(0);
    }

    public static Triplet<Spline, Point2d, Point2d> getRandomSplineAndEndpoints(){
        Point2d start = getRandomPoint(), end = getRandomPoint();
        RoadBuilder builder = new RoadBuilder(start, getRandomAngle(), 0, getRandomAngle());
        Road r = builder.spline(
                new Pose(end, getRandomAngle()),
                getRandomAngle(),
                getRandomAngle(),
                RoadBuilder.HeadingInterpMethod.SPLINE,
                false
        ).build();

        return new Triplet<>((Spline) r.getSegmentByIndex(0), start,end);
    }

    private static Road road1, road2, road3, road4, road5, road6, road7;
    static{
        RoadBuilder rb = new RoadBuilder(new Point2d(), 0, 0, 0);

        road1 = rb.line(new Point2d(3, -1)).spline(new Point2d(4, 5)).line(new Pose(10, 10, 1), FastMath.PI, RoadBuilder.HeadingInterpMethod.LINEAR).build();

        rb = new RoadBuilder(new Point2d(), 0, 0, 0);
        road2 = rb.spline(new Pose(3, 3, FastMath.PI / 2), FastMath.PI / 3, FastMath.PI, RoadBuilder.HeadingInterpMethod.LINEAR, false)
                .spline(new Point2d(5, 5))
                .spline(new Pose(10, 10, FastMath.PI), -FastMath.PI, -FastMath.PI, RoadBuilder.HeadingInterpMethod.SPLINE, true)
                .build();

        rb = new RoadBuilder(new Point2d(-1, -2), FastMath.PI, 0, 2d / 3d);
        road3 = rb.line(new Point2d(3, 4)).line(new Pose(5, 7, FastMath.PI / 4), FastMath.PI / 4, RoadBuilder.HeadingInterpMethod.SPLINE).build();

        rb = new RoadBuilder(new Point2d(1, -2), -FastMath.PI/6, 0, FastMath.PI/2);
        road4 = rb.spline(new Pose(-5, -5, -FastMath.PI / 2), FastMath.PI, -FastMath.PI, RoadBuilder.HeadingInterpMethod.SPLINE, false)
                .spline(new Pose(5, 5, 2), 1, 1, RoadBuilder.HeadingInterpMethod.LINEAR, false)
                .line(new Point2d(10, 10))
                .build();

        rb = new RoadBuilder(new Point2d(), 0, 0, FastMath.PI);
        road5 = rb.line(new Point2d(20, 20))
                .spline(new Point2d(-30, -30))
                .spline(new Pose(20, -15, FastMath.PI/18), FastMath.PI/4, 2d/5d, RoadBuilder.HeadingInterpMethod.LINEAR, false)
                .spline(new Pose(30, -20, FastMath.PI), 0, 0, RoadBuilder.HeadingInterpMethod.LINEAR, true)
                .build();

        rb = new RoadBuilder(new Point2d(), FastMath.PI / 3, 0, 0);
        road6 = rb.spline(new Pose(9, 9, 0), FastMath.PI, FastMath.PI, RoadBuilder.HeadingInterpMethod.SPLINE, false)
                .line(new Point2d(8, -8))
                .spline(new Point2d(-3, -3))
                .line(new Pose(5, -5, FastMath.PI), 1, RoadBuilder.HeadingInterpMethod.SPLINE)
                .build();

        rb = new RoadBuilder(new Point2d(), 0, 0);
        road7 = rb.line(new Point2d(3, 3)).spline(new Point2d(7, 6)).spline(new Point2d(12, 12)).build();
    }

    public static Pair<TrapezoidalMotionProfile, Road> getRandomProfile(ConstraintSet constr, int res, double xwheel, double ywheel){
        int a = (int) Math.round(FastMath.random() * 5);

        //a = 4;
        System.out.println("Selecting road number " + (a + 1));
        if(a == 0){
            return new Pair<>(
                    new TrapezoidalMotionProfile(
                            road1, constr, 0, 0, 0, xwheel, ywheel, res
                    ),
                    road1);
        } else if (a == 1){
            return new Pair<>(
                    new TrapezoidalMotionProfile(
                            road2, constr, 0, 0, 0, xwheel, ywheel, res
                    ),
                    road2);
        } else if (a == 2){
            return new Pair<>(
                    new TrapezoidalMotionProfile(
                            road3, constr, 0, 0, 0, xwheel, ywheel, res
                    ),
                    road3);
        } else if (a == 3){
            return new Pair<>(
                    new TrapezoidalMotionProfile(
                            road4, constr, 0, 0, 0, xwheel, ywheel, res
                    ),
                    road4);}
        else if (a == 4){
            return new Pair<>(
                    new TrapezoidalMotionProfile(
                            road5, constr, 0, 0, 0, xwheel, ywheel, res
                    ),
                    road5);
        } else if (a == 5) {
            return new Pair<>(
                    new TrapezoidalMotionProfile(
                            road6, constr, 0, 0, 0, xwheel, ywheel, res
                    ),
                    road6);
        } else {
            return new Pair<>(
                    new TrapezoidalMotionProfile(
                            road7, constr, 0, 0, 0, xwheel, ywheel, res
                    ),
                    road7);
        }
    }

    public static ConstraintSet getRandomConstraints(){
        double maxVel = FastMath.random() * 15 + 5;
        double maxAcc = FastMath.random() * 22 + 8;
        double trackWidth = FastMath.random() * 9 + 6;

        double maxAngVel = maxVel / trackWidth;
        double maxAngAcc = maxAcc / trackWidth;

        return new ConstraintSet(maxVel, maxAcc, maxAngVel, maxAngAcc);
    }
}
