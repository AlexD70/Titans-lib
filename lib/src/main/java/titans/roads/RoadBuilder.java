package titans.roads;

import org.apache.commons.math3.util.FastMath;
import titans.math.NPoly;
import titans.geometry.Line2d;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.math.Spline;
import titans.util.Unimplemented;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

public class RoadBuilder {
    private Point2d prevPoint = null;
    private Point2d startPoint;
    private double startDeriv;
    private double start2ndDeriv;
    private double startAng = 0; // this means 1i + 0j
    private double lastAngDeriv = 0;
    private Road road = new Road();
    private double eps  = 0.001;

    public RoadBuilder(Point2d startPoint, double startDeriv, double start2ndDeriv){
        this.startPoint = startPoint;
        this.startDeriv = startDeriv;
        this.start2ndDeriv = start2ndDeriv;
    }

    public RoadBuilder(Point2d startPoint, double startDeriv, double start2ndDeriv, double startAng) {
        this.startPoint = startPoint;
        this.startDeriv = startDeriv;
        this.start2ndDeriv = start2ndDeriv;
        this.startAng = startAng;
    }

    @SuppressWarnings("unused")
    public RoadBuilder strafe(double howmuch){
        return goByDirections(howmuch, 0);
    }

    @SuppressWarnings("unused")
    public RoadBuilder forward(double howmuch){
        return goByDirections(0, howmuch);
    }

    @SuppressWarnings("unused")
    public RoadBuilder goByDirections(double leftRight, double forwardBackward){
        return lineConstHeading(new Point2d(prevPoint.getX() + leftRight, prevPoint.getY() + forwardBackward));
    }

    // the original heuristic uses 1/2 * lastDist for the last spline, however the usage
    // we seek doesn't quite allow that, so let's leave it at max(dist1, dist2)
    @Nonnull private Vector2d tangentVector(double startTangent, @Nonnull Point2d start, @Nonnull Point2d end, @Nullable Point2d prev) {
        double r = 1;

        if(prev == null) {
            r = Point2d.dist(start, end);
        } else {
            r = 2 * FastMath.max(Point2d.dist(start, end), Point2d.dist(prev, start));
        }

        return Vector2d.fromPolar(r, startTangent);
    }

    @SuppressWarnings("unused")
    public RoadBuilder line(Point2d endPoint){
        return this.lineConstHeading(endPoint);
    }

    @SuppressWarnings("unused")
    public RoadBuilder spline(Point2d endPoint){
        return this.splineConstHeading(endPoint);
    }

    @SuppressWarnings("unused")
    public RoadBuilder turn(double endAngle){
        HeadingInterpolation rotation = HeadingInterpolation.fromAngles(
                Vector2d.fromPolar(1, startAng), endAngle - startAng, 0, 0, true
        );
        startAng = endAngle;
        road.addSplineWithHeadingInterp(new Spline(), false, rotation);
        return this;
    }

    // line parameterization
    @SuppressWarnings("unused")
    public RoadBuilder lineConstHeading(Point2d endPoint){
        Vector2d v = endPoint.toVector().diff(startPoint.toVector());
        double[] xcoeffs = new double[] {0, 0, 0, 0, v.getX(), startPoint.getX()};
        double[] ycoeffs = new double[] {0, 0, 0, 0, v.getY(), startPoint.getY()};

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
        road.addSpline(s, cont);

        startDeriv = Line2d.getSlope(startPoint, endPoint);
        start2ndDeriv = 0;
        prevPoint = startPoint;
        startPoint = endPoint;

        return this;
    }

    @SuppressWarnings("unused")
    public RoadBuilder splineConstHeading(Point2d endPoint){
        Vector2d endTangentVector = tangentVector(startDeriv, startPoint, endPoint, prevPoint);
        Point2d tangentVectorEndPoint = endTangentVector.toPoint();

        Spline s = new Spline(startPoint, endPoint, tangentVectorEndPoint, tangentVectorEndPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s, true);

        return this;
    }

    @SuppressWarnings("unused")
    public RoadBuilder splineConstHeading(Point2d endPoint, double endTangent){
        Point2d startTangentPoint = tangentVector(startDeriv, startPoint, endPoint, prevPoint).toPoint();
        Point2d endTangentPoint = tangentVector(endTangent, startPoint, endPoint, prevPoint).toPoint();

        Spline s = new Spline(startPoint, endPoint, startTangentPoint, endTangentPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        startDeriv = endTangent;
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s, true);

        return this;
    }

    @SuppressWarnings("unused")
    public RoadBuilder lineLinearHeading(Point2d endPoint, double endAngle){
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

    @SuppressWarnings("unused")
    public RoadBuilder lineSplineHeading(Point2d endPoint, double endAngle, double endTangentHeading){
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

    @SuppressWarnings("unused")
    public RoadBuilder splineLinearHeading(Point2d endPoint, double endAngle){
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

    @SuppressWarnings("unused")
    public RoadBuilder splineLinearHeading(Point2d endPoint, double endAngle, double endTangent){
        return this;
    }

    @SuppressWarnings("unused")
    public RoadBuilder splineSplineHeading(Point2d endPoint, double endAngle, double endTangent, double endTangentHeading){
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

    @SuppressWarnings("unused")
    public Road build(){
        return road;
    }
}
