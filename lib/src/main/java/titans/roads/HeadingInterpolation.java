package titans.roads;

import titans.math.HermiteInterpolation;
import titans.math.NPoly;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.math.Spline;

public class HeadingInterpolation {
    public boolean isLinear = true, isConst = false;
    public Vector2d startHeading, endHeading;

    public Spline interp = null;
    public HermiteInterpolation hermite = null;

    public HeadingInterpolation() {
        isConst = true;
        isLinear = false;
    }

    public static HeadingInterpolation fromAngles(Vector2d startHeading, double deltaAng, double startTan, double endTan, boolean isLinear) {
        HeadingInterpolation interpolation = new HeadingInterpolation();
        interpolation.startHeading = startHeading;
        interpolation.endHeading = startHeading.rotate(deltaAng); // fix me
        Vector2d delta = interpolation.endHeading.diff(startHeading);
        interpolation.isConst = false;

        if (isLinear) {
            double[] xcoeffs = new double[]{0, 0, 0, 0, delta.getX(), startHeading.getX()};
            double[] ycoeffs = new double[]{0, 0, 0, 0, delta.getY(), startHeading.getY()};

            interpolation.interp = new Spline();
            NPoly xpoly = new NPoly(5), ypoly = new NPoly(5);
            xpoly.assignCoefficients(xcoeffs);
            ypoly.assignCoefficients(ycoeffs);
            interpolation.interp.setSpline(xpoly, ypoly, startHeading.toPoint());
            interpolation.interp.unsetNull();
            interpolation.isLinear = true;
        } else {
            //double dist = Point2d.dist(startHeading.toPoint(), interpolation.endHeading.toPoint());
            //Point2d startDeriv = Vector2d.fromPolar(dist, startTan).toPoint();
            //Point2d endDeriv = Vector2d.fromPolar(dist, endTan).toPoint();
            startHeading.toPolar();
            interpolation.hermite = new HermiteInterpolation(startHeading.getT(), startHeading.getT() + deltaAng, startTan, endTan);
            //interpolation.interp = Spline.buildSpline6(startHeading.toPoint(), interpolation.endHeading.toPoint(), startDeriv, endDeriv, new Point2d(), new Point2d());
            interpolation.isLinear = false;
        }


        return interpolation;
    }

    public Vector2d getVectorAtParam(double u) {
        if(!isLinear){
            double ang = hermite.apply(u);
            return Vector2d.fromPolar(1, ang);
        }

        Vector2d ret = this.interp.pointAt(u).toVector();
        ret.toPolar();
        return ret;
    }

    public Vector2d getNormedVectorAtParam(double u) {
        if(!isLinear){
            return getVectorAtParam(u);
        }

        Vector2d v = getVectorAtParam(u).norm();
        v.toPolar();
        return v;
    }
}
