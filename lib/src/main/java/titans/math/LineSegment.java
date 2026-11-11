package titans.math;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.jetbrains.annotations.Contract;
import titans.geometry.Line2d;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.util.DispOfBoundsException;

import javax.annotation.Nonnull;

/**
 * This class represents a line segment for a path, implementing the {@link ParametricCurve}
 * interface and managing internally a {@link Line2d} for line parametrization between endpoints.
 *
 * @author AlexD70
 */
public class LineSegment implements ParametricCurve {
    private final Point2d startPoint, endPoint;
    private final Line2d line;
    private final double length;

    public LineSegment(@Nonnull Point2d startPoint, @Nonnull Point2d endPoint){
        line = new Line2d(startPoint, endPoint);
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        length = Point2d.dist(startPoint, endPoint);
    }

    @Override @Contract(value = "-> _", pure = true)
    public @NonNull Point2d getStartPoint() {
        return startPoint;
    }

    @Override @Contract(value = "-> _", pure = true)
    public @NonNull Point2d getEndPoint() {
        return endPoint;
    }

    @Override
    public double getLength() {
        return length;
    }

    /**
     * Returns the point on the line at a given internal parameter u. For u = 0, it should return a point that equals
     * the output of {@link #getStartPoint()} and for u = 1 the output should match {@link #getEndPoint()} (in the sense of {@link Point2d#equals(Object)}).
     * @param u the internal parameter in [0, 1]
     * @throws DispOfBoundsException if the internal parameter is not in the right range
     * @return a {@code new Point2d}
     */
    @Override @Contract(value = "_ -> new", pure = true)
    public @NonNull Point2d pointAt(double u) {
        if(!MathUtils.dispInRange(u, 0, 1)){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        if(u == 0){
            return startPoint;
        } else if (u == 1){
            return endPoint;
        }

        return line.pointAtParam(u);
    }

    /**
     * Returns the displacement from the segment start point to the point at the given parameter (Euclidean distance).
     * @throws DispOfBoundsException if the internal parameter is not in the right range
     * @param u the internal parameter in [0, 1].
     * @return the displacement
     */
    @Override
    public double dispAt(double u) {
        if(!MathUtils.dispInRange(u, 0, 1)){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        return u * length;
    }

    /**
     * Returns the derivative of the line.
     * @throws DispOfBoundsException if the internal parameter is not in the right range
     * @param u the internal parameter in [0, 1]
     * @return a {@code new Point2d} containing (x'(t), y'(t))
     */
    @Override @Contract(value = "_ -> new", pure = true)
    public @NonNull Point2d derivAt(double u) {
        if(!MathUtils.dispInRange(u, 0, 1)){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        return new Point2d(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
    }

    /**
     * Returns the 2nd derivative of the line.
     * @throws DispOfBoundsException if the internal parameter is not in the right range
     * @param u the internal parameter in [0, 1]
     * @return a {@code new Point2d} containing (0, 0)
     */
    @Override @Contract(value = "_ -> new", pure = true)
    public @NonNull Point2d deriv2ndAt(double u) {
        if(!MathUtils.dispInRange(u, 0, 1)){
            throw new DispOfBoundsException(u, 0, 1, this);
        }

        return new Point2d(0, 0);
    }

    /**
     * Returns the angle of the tangent vector at the line.
     * @param u the internal parameter in [0, 1]
     * @return the angle between -pi and pi
     */
    @Override
    public double tanAt(double u) {
        return derivAt(u).toVector().getT();
    }

    @Override @Contract(value = "_ -> new", pure = true)
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

    @Override @Contract(value = "_ -> new", pure = true)
    public @NonNull Point2d pointAtDisp(double d) {
        return pointAt(uAtDisp(d));
    }

    @Override
    public String toString() {
        return super.toString() + ":\n\tstartPoint " + startPoint + "\n\tendPoint " + endPoint + "\n\tline " + line;
    }
}
