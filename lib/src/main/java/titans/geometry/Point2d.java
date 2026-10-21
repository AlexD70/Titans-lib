package titans.geometry;

import org.apache.commons.math3.util.FastMath;

/**
 * An immutable 2d point object providing various functionality and acting as a container for 2 doubles.
 *
 * <p>
 *     Has various operations such as 2d translation,
 *     distance, and even difference between 2 points (element wise difference).
 *     Provides the following conversions to other geometry classes:
 * </p>
 * <ul>
 *     <li>To {@code Vector2d} through {@code toVector()}</li>
 *     <li>To {@code Pose} through {@code addAngle(double)}</li>
 * </ul>
 *
 *
 * @author AlexD70
 * @see Vector2d
 * @see Pose
 */
public class Point2d {
    double x = 0, y = 0;

    /**
     * Builds the origin of the plane point (0, 0)
     */
    public Point2d(){}

    /**
     * Builds a point with given coordinates. Check if a {@code Vector2d} is better for your use case.
     * @see Vector2d
     */
    public Point2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    /**
     * Checks if both coordinates of the points are within a {@code 1e-4} of each other.
     *
     * @return the result of the test mentioned
     */
    @Override
    public boolean equals(Object obj) {
        if(!(obj instanceof Point2d)){
            return false;
        }
        return (FastMath.abs(((Point2d)obj).x - x) < 0.0001) && (FastMath.abs(((Point2d)obj).y - y) < 0.0001);
    }

    /**
     * This function provides a way to directly convert a {@code Point2d} to a {@code Pose} by
     * appending an angle to the coordinates.
     *
     * @param angle
     * @return a new {@code Pose} object
     *
     * @see Pose
     */
    public Pose addAngle(double angle){
        return new Pose(this, angle);
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }

    /**
     * This function provides a conversion to {@code Vector2d}. The new vector has both
     * cartesian and polar representations.
     * @return a new {@code Vector2d} object
     */
    public Vector2d toVector(){
        return new Vector2d(x, y);
    }

    /**
     * This function returns a new point translated by a {@code Vector2d}.
     * @param translate the vector to translate by
     * @return a new {@code Point2d} object
     */
    public Point2d translate(Vector2d translate){
        return new Point2d(this.x + translate.getX(), this.y + translate.getY());
    }

    /**
     * This function builds the vector AB from its startpoint and endpoint.
     * @return the distance vector from {@code a} to {@code b}
     */
    public static Vector2d distVector(Point2d a, Point2d b){
        return Vector2d.fromPolar(dist(a, b), Line2d.getSlope(a, b));
    }

    /**
     * An alias for {@code distVector(Point2d, Point2d)}
     *
     * @see #distVector(Point2d, Point2d)
     */
    public Vector2d diff(Point2d other){
        return distVector(this, other);
    }

    /**
     * A function for the scalar distance.
     * @return the distance between {@code a} and {@code b}
     */
    public static double dist(Point2d a, Point2d b){
        return FastMath.sqrt(FastMath.pow(a.x - b.x, 2) + FastMath.pow(a.y - b.y, 2));
    }

    @Override
    public String toString() {
        return super.toString() + String.format(" (%f, %f)", x, y);
    }
}
