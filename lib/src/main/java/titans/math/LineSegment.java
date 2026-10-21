package titans.math;

import org.checkerframework.checker.nullness.qual.NonNull;
import titans.geometry.Line2d;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.util.DispOfBoundsException;

public class LineSegment implements ParametricCurve {
    Point2d startPoint = null, endPoint = null;
    Line2d line = null;
    double length = -1;

    public LineSegment(Point2d startPoint, Point2d endPoint){
        line = new Line2d(startPoint, endPoint);
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        length = Point2d.dist(startPoint, endPoint);
    }

    @Override
    public @NonNull Point2d getStartPoint() {
        return startPoint;
    }

    @Override
    public @NonNull Point2d getEndPoint() {
        return endPoint;
    }

    @Override
    public double getLength() {
        return length;
    }

    @Override
    public @NonNull Point2d pointAt(double u) {
        if(u < 0 || u > 1){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        if(u == 0){
            return startPoint;
        } else if (u == 1){
            return endPoint;
        }

        return line.pointAtParam(u);
    }

    @Override
    public double dispAt(double u) {
        if(u < 0 || u > 1){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        return u * length;
    }

    @Override
    public @NonNull Point2d derivAt(double u) {
        if(u < 0 || u > 1){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        return new Point2d(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
    }

    @Override
    public @NonNull Point2d deriv2ndAt(double u) {
        if(u < 0 || u > 1){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        return new Point2d(0, 0);
    }

    @Override
    public double tanAt(double u) {
        return derivAt(u).toVector().getT();
    }

    @Override
    public @NonNull Vector2d tanVecAt(double u) {
        return derivAt(u).toVector().norm();
    }

    @Override
    public double uAtDisp(double d) {
        if(!MathUtils.dispInRange(d, 0, length)){
            throw new DispOfBoundsException(d, 0, length, this);
        }

        return d / length;
    }

    @Override
    public @NonNull Point2d pointAtDisp(double d) {
        return pointAt(uAtDisp(d));
    }

    @Override
    public String toString() {
        return super.toString() + ":\n\tstartPoint " + startPoint + "\n\tendPoint " + endPoint + "\n\tline " + line;
    }
}
