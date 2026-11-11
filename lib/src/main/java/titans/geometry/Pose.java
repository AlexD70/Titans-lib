package titans.geometry;

import org.apache.commons.math3.util.FastMath;
import org.jetbrains.annotations.Contract;

import javax.annotation.Nonnull;

/**
 * This class represents a point paired with a heading practically being the full robot position in 2D.
 * After building the object, it is immutable.
 *
 * @author AlexD70
 */
public class Pose {
    private final Point2d point;
    private final double theta;

    public Pose(double x, double y, double theta) {
        this.point = new Point2d(x, y);
        this.theta = theta;
    }

    public Pose(@Nonnull Point2d p, double theta){
        this.point = p;
        this.theta = theta;
    }

    public double getX(){
        return point.getX();
    }

    public double getY(){
        return point.getY();
    }

    public double getTheta(){
        return this.theta;
    }

    @Nonnull @Contract("-> _")
    public Point2d getPoint(){
        return point;
    }

    /**
     * Converts the position into a position vector and returns it.
     * @return the position vector as a {@code new Vector2d}
     */
    @Nonnull @Contract("-> new")
    public Vector2d getVector(){
        return point.toVector();
    }

    private static final double EPS = 0.001;
    @Override
    public boolean equals(Object obj){
        if(obj instanceof Pose pose){
            return (pose.point.equals(this.point) && (FastMath.abs(pose.theta - this.theta) < EPS));
        } else {
            return false;
        }
    }

    @Override
    public String toString() {
//        if(point == null){
//            return super.toString() + String.format(" (NULL %frad)", getTheta());
//        }
        return super.toString() + String.format(" (%f %f %frad)", getX(), getY(), getTheta());
    }
}
