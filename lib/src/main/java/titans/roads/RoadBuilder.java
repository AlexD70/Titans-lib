package titans.roads;

import org.apache.commons.math3.util.FastMath;
import titans.geometry.Pose;
import titans.math.LineSegment;
import titans.math.NPoly;
import titans.geometry.Line2d;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.math.Spline;
import titans.util.Unimplemented;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * This class provides a way for users to build a {@link Road} in a simple manner.
 * The class uses heuristics to determine certain parameters required for building {@code ParametricCurves}
 * such as {@code Spline}s and also builds {@code HeadingInterpolation}s using as little parameters as possible, before adding
 * them as segments to the road.
 *
 * <p>
 *     This is intended for single-use. Once the road is built, a new {@code RoadBuilder} should be initialized to
 *     build a new {@code Road}. The curve-building functions can be chained until the {@link #build()} function is called to
 *     return the final {@code Road} object.
 * </p>
 *
 * @author AlexD70
 * @see Road
 * @see Spline
 * @see HeadingInterpolation
 */
public class RoadBuilder {
    private Point2d prevPoint = null;
    private Point2d startPoint;
    private double startDeriv;
    private double start2ndDeriv;
    private double startAng = 0; // this means 1i + 0j
    private double lastAngDeriv = 0;
    private final Road road = new Road();
    private double eps  = 0.001;

    /**
     * Initializes a new builder with {@code startAng = 0} as the starting angle.
     * @param startPoint the starting point of the {@code Road}
     * @param startDeriv the start tangent, in radians
     * @param start2ndDeriv the start second tangent, in radians
     */
    public RoadBuilder(Point2d startPoint, double startDeriv, double start2ndDeriv){
        this.startPoint = startPoint;
        this.startDeriv = startDeriv;
        this.start2ndDeriv = start2ndDeriv;
        this.prevPoint = startPoint;
    }

    /**
     * Initializes a new builder with.
     * @param startPoint the starting point of the {@code Road}
     * @param startDeriv the start tangent, in radians
     * @param start2ndDeriv the start second tangent, in radians
     * @param startAng the start angle, in radians
     */
    public RoadBuilder(Point2d startPoint, double startDeriv, double start2ndDeriv, double startAng) {
        this.startPoint = startPoint;
        this.startDeriv = startDeriv;
        this.start2ndDeriv = start2ndDeriv;
        this.startAng = startAng;
    }

    /**
     * This enumeration describes the methods of interpolating headings with the {@code ParametricCurve}s.
     * The methods are: {@code CONSTANT}, {@code LINEAR} and {@code SPLINE}.
     */
    public enum HeadingInterpMethod {
        CONSTANT(0), LINEAR(1), SPLINE(2);

        final int id;
        HeadingInterpMethod(int id){
            this.id = id;
        }

    }

    /**
     * This is a function for building a line segment to a given {@code Pose} using the specified
     * {@code HeadingInterpMethod}. Certain parameters will be ignored by the function depending on the value of
     * the {@code flag} parameter.
     *
     * @param endPose a {@code Pose}. The angle should be in radians. The angle does not matter if {@code flag == HeadingInterpMethod.CONSTANT}
     * @param endTanHeading the tangent of the heading interpolation, in radians. It is ignored if {@code flag == HeadingInterpMethod.CONSTANT || flag == HeadingInterpMethod.LINEAR}
     * @param flag the {@link HeadingInterpMethod} to use
     * @return the object itself, for chaining
     * @throws RuntimeException when an invalid {@code flag} is passed
     */
    @SuppressWarnings("unused")
    public RoadBuilder line(Pose endPose, double endTanHeading, HeadingInterpMethod flag){
        if(flag == HeadingInterpMethod.CONSTANT){
            return this.lineConstHeading(endPose.getPoint());
        } else if (flag == HeadingInterpMethod.LINEAR){
            return this.lineLinearHeading(endPose.getPoint(), endPose.getTheta());
        } else if (flag == HeadingInterpMethod.SPLINE){
            return this.lineSplineHeading(endPose.getPoint(), endPose.getTheta(), endTanHeading);
        } else {
            throw new RuntimeException("Unknown heading interpolation method");
        }
    }

    /**
     * This is a function for building a spline segment to a given {@code Pose} using the specified
     * {@code HeadingInterpMethod}. Certain parameters will be ignored by the function depending on the value of
     * the {@code flag} and {@code keepEndTangent} parameters.
     *
     * @param endPose a {@code Pose}. The angle should be in radians. The angle does not matter if {@code flag == HeadingInterpMethod.CONSTANT}
     * @param endTan the end tangent of the spline, in radians. It is ignored if {@code keepEndHeading == true}
     * @param endTanHeading the tangent of the heading interpolation, in radians. It is ignored if {@code flag == HeadingInterpMethod.CONSTANT || flag == HeadingInterpMethod.LINEAR} or {@code keepEndTangent == true}
     * @param flag the {@link HeadingInterpMethod} to use
     * @param keepEndTangent a boolean indicating if the end tangent should change
     * @return the object itself, for chaining
     * @throws RuntimeException when an invalid {@code flag} is passed
     */
    @SuppressWarnings("unused")
    public RoadBuilder spline(Pose endPose, double endTan, double endTanHeading, HeadingInterpMethod flag, boolean keepEndTangent) {
        if(keepEndTangent) {
            if (flag == HeadingInterpMethod.CONSTANT) {
                return this.splineConstHeading(endPose.getPoint());
            } else if (flag == HeadingInterpMethod.LINEAR) {
                return this.splineLinearHeading(endPose.getPoint(), endPose.getTheta());
            } else if (flag == HeadingInterpMethod.SPLINE) {
                return this.splineSplineHeading(endPose.getPoint(), endPose.getTheta(), startDeriv, lastAngDeriv);
            } else {
                throw new RuntimeException("Unknown heading interpolation method");
            }
        } else {
            if (flag == HeadingInterpMethod.CONSTANT) {
                return this.splineConstHeading(endPose.getPoint(), endTan);
            } else if (flag == HeadingInterpMethod.LINEAR) {
                return this.splineLinearHeading(endPose.getPoint(), endPose.getTheta(), endTan);
            } else if (flag == HeadingInterpMethod.SPLINE) {
                return this.splineSplineHeading(endPose.getPoint(), endPose.getTheta(), endTan, endTanHeading);
            } else {
                throw new RuntimeException("Unknown heading interpolation method");
            }
        }
    }

    // the original heuristic uses 1/2 * lastDist for the last spline, however the usage
    // we seek doesn't quite allow that, so let's leave it at max(dist1, dist2)

    /**
     * This function is a heuristic for the tangent vector of a spline built from the endpoints of the spline,
     * the start tangent angle and the startpoint of the previous segment.
     *
     * <p>
     *     If the spline built is the first segment in the {@code Road}, then the resulting vector will have the
     *     absolute value equal to the euclidean distance between the spline endpoints. Otherwise, the absolute value will be
     *     twice the max between the euclidean distances between the spline endpoints and between the previous segment's endpoints.
     * </p>
     * @param startTangent the start tangent, in radians
     * @param start the startpoint of the spline
     * @param end the endpoint of the spline
     * @param prev the startpoint of the previous segment
     * @return a new {@code Vector2d} object representing the end tangent vector of the spline
     */
    @Nonnull private Vector2d tangentVector(double startTangent, @Nonnull Point2d start, @Nonnull Point2d end, @Nullable Point2d prev) {
        double r = 1;

        if(prev == null) {
            r = Point2d.dist(start, end);
        } else {
            r = 2 * FastMath.max(Point2d.dist(start, end), Point2d.dist(prev, start));
        }

        return Vector2d.fromPolar(r, startTangent);
    }

    /**
     * A shortcut for building a line with a constant heading.
     * @param endPoint the end point
     * @return the object itself, for chaining
     */
    @SuppressWarnings("unused")
    public RoadBuilder line(Point2d endPoint){
        return this.lineConstHeading(endPoint);
    }

    /**
     * A shortcut for building a spline with a constant heading.
     * @param endPoint the end point
     * @return the object itself, for chaining
     */
    @SuppressWarnings("unused")
    public RoadBuilder spline(Point2d endPoint){
        return this.splineConstHeading(endPoint);
    }

    /**
     * Builds a pure turn segment to a given target angle.
     * @param endAngle the target angle in radians
     * @return the object itself, for chaining
     */
    @SuppressWarnings("unused")
    public RoadBuilder turn(double endAngle){
        HeadingInterpolation rotation = HeadingInterpolation.fromAngles(
                Vector2d.fromPolar(1, startAng), endAngle - startAng, 0, 0, true
        );
        startAng = endAngle;
        road.addSplineWithHeadingInterp(new Spline(), false, rotation);
        return this;
    }

    /**
     * Creates a line segment with constant heading using line parameterization.
     * @param endPoint the end point
     * @return the object itself, for chaining
     */
    protected RoadBuilder lineConstHeading(Point2d endPoint){
        Vector2d v = endPoint.toVector().diff(startPoint.toVector());
        LineSegment line = new LineSegment(prevPoint, endPoint);

        boolean cont = FastMath.abs(startDeriv - Line2d.getSlope(startPoint, endPoint)) < eps;
        road.addSpline(line, cont);

        startDeriv = line.tanAt(1);
        start2ndDeriv = 0;
        prevPoint = startPoint;
        startPoint = endPoint;

        return this;
    }

    /**
     * Creates a spline segment with constant heading, while keeping tangent.
     * @param endPoint the end point
     * @return the object itself, for chaining
     */
    protected RoadBuilder splineConstHeading(Point2d endPoint){
        Vector2d endTangentVector = tangentVector(startDeriv, startPoint, endPoint, prevPoint);
        Point2d tangentVectorEndPoint = endTangentVector.toPoint();

        Spline s = new Spline(startPoint, endPoint, tangentVectorEndPoint, tangentVectorEndPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s, true);

        return this;
    }


    /**
     * Creates a spline segment with constant heading.
     * @param endPoint the end point
     * @param endTangent the new end tangent, in radians
     * @return the object itself, for chaining
     */
    protected RoadBuilder splineConstHeading(Point2d endPoint, double endTangent){
        Point2d startTangentPoint = tangentVector(startDeriv, startPoint, endPoint, prevPoint).toPoint();
        Point2d endTangentPoint = tangentVector(endTangent, startPoint, endPoint, prevPoint).toPoint();

        Spline s = new Spline(startPoint, endPoint, startTangentPoint, endTangentPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        startDeriv = endTangent;
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s, true);

        return this;
    }

    /**
     * Creates a line segment with linear heading interpolation.
     * @param endPoint the end point
     * @param endAngle the target angle, in radians
     * @return the object itself, for chaining
     */
    protected RoadBuilder lineLinearHeading(Point2d endPoint, double endAngle){
        Vector2d v = endPoint.toVector().diff(startPoint.toVector());
        double[] xcoeffs = new double[] {0, 0, 0, 0, v.getX(), startPoint.getX()};
        double[] ycoeffs = new double[] {0, 0, 0, 0, v.getY(), startPoint.getY()};

        HeadingInterpolation splineInterp = HeadingInterpolation.fromAngles(
                Vector2d.fromPolar(1, startAng), endAngle - startAng, 0, 0, true
        );
        startAng = endAngle;

        Spline s = new Spline();
        NPoly xpoly = new NPoly(5), ypoly =  new NPoly(5);
        xpoly.assignCoefficients(xcoeffs);
        ypoly.assignCoefficients(ycoeffs);
        s.setSpline(xpoly, ypoly, prevPoint);
        s.unsetNull();
        try {
            s.unsetNull();
        } catch (Exception e){
            e.printStackTrace();
            System.exit(-1);
        }

        boolean cont = FastMath.abs(startDeriv - Line2d.getSlope(startPoint, endPoint)) < eps;
        road.addSplineWithHeadingInterp(s, cont, splineInterp);

        startDeriv = Line2d.getSlope(startPoint, endPoint);
        start2ndDeriv = 0;
        prevPoint = startPoint;
        startPoint = endPoint;

        return this;
    }

    /**
     * Creates a line segment with spline heading interpolation, that does not keep the heading tangent.
     * @param endPoint the end point
     * @param endAngle the target angle, in radians
     * @param endTangentHeading the end tangent of the heading interpolation, in radians
     * @return the object itself, for chaining
     */
    protected RoadBuilder lineSplineHeading(Point2d endPoint, double endAngle, double endTangentHeading){
        Vector2d v = endPoint.toVector().diff(startPoint.toVector());
        double[] xcoeffs = new double[] {0, 0, 0, 0, v.getX(), startPoint.getX()};
        double[] ycoeffs = new double[] {0, 0, 0, 0, v.getY(), startPoint.getY()};
        HeadingInterpolation splineInterp = HeadingInterpolation.fromAngles(
                Vector2d.fromPolar(1, startAng), endAngle - startAng, lastAngDeriv, endTangentHeading, false
        );
        startAng = endAngle;

        Spline s = new Spline();
        NPoly xpoly = new NPoly(5), ypoly =  new NPoly(5);
        xpoly.assignCoefficients(xcoeffs);
        ypoly.assignCoefficients(ycoeffs);
        s.setSpline(xpoly, ypoly, prevPoint);
        s.unsetNull();
        try {
            s.unsetNull();
        } catch (Exception e){
            e.printStackTrace();
            System.exit(-1);
        }

        boolean cont = FastMath.abs(startDeriv - Line2d.getSlope(startPoint, endPoint)) < eps;
        road.addSplineWithHeadingInterp(s, cont, splineInterp);

        startDeriv = Line2d.getSlope(startPoint, endPoint);
        start2ndDeriv = 0;
        prevPoint = startPoint;
        startPoint = endPoint;

        return this;
    }

    /**
     * Creates a spline segment with linear heading interpolation while keeping tangent.
     * @param endPoint the end point
     * @param endAngle the target angle, in radians
     * @return the object itself, for chaining
     */
    protected RoadBuilder splineLinearHeading(Point2d endPoint, double endAngle){
        Vector2d endTangentVector = tangentVector(startDeriv, startPoint, endPoint, prevPoint);
        Point2d tangentVectorEndPoint = endTangentVector.toPoint();
        HeadingInterpolation splineInterp = HeadingInterpolation.fromAngles(
                Vector2d.fromPolar(1, startAng), endAngle - startAng, 0, 0, true
        );
        startAng = endAngle;

        Spline s = new Spline(startPoint, endPoint, tangentVectorEndPoint, tangentVectorEndPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSplineWithHeadingInterp(s, true, splineInterp);

        return this;
    }

    /**
     * !TODO:Unimplemented
     * @param endPoint the end point
     * @param endAngle the end angle, in radians
     * @param endTangent the end tangent, in radians
     * @return
     */
    @SuppressWarnings("unused")
    public RoadBuilder splineLinearHeading(Point2d endPoint, double endAngle, double endTangent){
        return this;
    }

    /**
     * !TODO: fixme
     * Creates a spline segment with spline heading interpolation.
     * @param endPoint the end point
     * @param endAngle the target angle, in radians
     * @param endTangent the end tangent of the spline, in radians
     * @param endTangentHeading the end tangent of the heading interpolation, in radians
     * @return the object itself, for chaining
     */
    protected RoadBuilder splineSplineHeading(Point2d endPoint, double endAngle, double endTangent, double endTangentHeading){
        Vector2d endTangentVector = tangentVector(startDeriv, startPoint, endPoint, prevPoint);
        Point2d tangentVectorEndPoint = endTangentVector.toPoint();
        HeadingInterpolation splineInterp = HeadingInterpolation.fromAngles(
                Vector2d.fromPolar(1, startAng), endAngle - startAng, lastAngDeriv, endTangentHeading, false
        );
        startAng = endAngle;

        Spline s = new Spline(startPoint, endPoint, tangentVectorEndPoint, tangentVectorEndPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        prevPoint = startPoint;
        startPoint = endPoint;
        lastAngDeriv = endTangentHeading;
        road.addSplineWithHeadingInterp(s, true, splineInterp);

        return this;
    }

    // ???
    @SuppressWarnings("unused")
    @Unimplemented
    public RoadBuilder splineAlter2ndDeriv(Point2d endPoint, double deriv2){
        return this;
    }

    /**
     * Finishes the building chain by returning the final {@code Road} object.
     * @return the built {@code Road} object
     * @see Road
     */
    @SuppressWarnings("unused")
    public Road build(){
        return road;
    }
}
