package titans.geometry;

import org.apache.commons.math3.util.FastMath;
import org.javatuples.Pair;
import org.jetbrains.annotations.Contract;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * This class represents a line in 2D space.
 *
 * @author AlexD70
 */
public class Line2d {
    private final double slope;
    private final Point2d a, b;

    /**
     * Builds a new {@code Line2D} object. The ctor fails if the points are the same.
     * @param a a point
     * @param b another point
     * @throws IllegalArgumentException if {@code a.equals(b)}
     * @see Point2d#equals(Object)
     */
    public Line2d(@Nonnull Point2d a, @Nonnull Point2d b){
        if(a.equals(b)){
            throw new IllegalArgumentException(String.format("Line object cannot be created from identical points: %s, %s.", a.toString(), b.toString()));
        }

        this.a = a;
        this.b = b;

        slope = FastMath.atan2(b.y - a.y, b.x - a.x);
    }

    /**
     * Returns the point at a given internal parameter computed for the parametrization of the line.
     * The line is automatically parameterized such that the first ctor argument is at u = 0 and the second
     * is at u = 1: x(u) = ax + u * (bx - ax), y(u) = ay + u * (by - ay)
     * @param u the internal parameter
     * @return a {@code new Point2d}
     */
    @Nonnull @Contract(value = "_ -> new", pure = true)
    public Point2d pointAtParam(double u){
        return new Point2d((a.x + u * (b.x - a.x)), (a.y + u * (b.y - a.y)));
    }

    @Contract("_, _ -> _")
    public static double getSlope(@Nonnull Point2d a, @Nonnull Point2d b){
        return (new Line2d(a, b)).slope;
    }

    @Contract("_, _ -> _")
    public static double getLineAngle(@Nonnull Point2d a, @Nonnull Point2d b){
        return FastMath.atan2(a.getY() - b.getY(), a.getX() - b.getX());
    }

    // solves the equation system:
    // x = xA + t(xB - xA)
    // y = yA + t(yB - yA)
    // (x - xC)^2 + (y - yC)^2 = R^2
    // if there are no intersection points, this returns null
    //private Pair<Double, Double> cachedT = null;
    //private Circle2d tCachedFor = null;
    @Nullable
    public Pair<Double, Double> intersect_Circle(@Nonnull Circle2d c){
        double dx = b.x - a.x;
        double dy = b.y - a.y;

        // AX^2 + BX + C = 0
        double freeTerm = FastMath.pow(a.x - c.getCX(), 2) + FastMath.pow(a.y - c.getCY(), 2) - FastMath.pow(c.getR(), 2); // C
        double bTerm = 2 * (dx * (a.x - c.getCX()) + dy * (a.y - c.getCY())); // B
        double aTerm = dx * dx + dy * dy; // A

        double discriminant = FastMath.pow(bTerm, 2) - 4 * aTerm * freeTerm;
        if(discriminant < 0){
            return null;
        } else if (discriminant == 0){
            double t = - bTerm / (2 * aTerm);
            //cachedT = new Pair<>(t, null);
            //tCachedFor = c;

            return new Pair<>(t, null);
        } else {
            double sqrtDisc = FastMath.sqrt(discriminant);
            double t1 = (-bTerm + sqrtDisc) / (2 * aTerm);
            double t2 = (-bTerm - sqrtDisc) / (2 * aTerm);
            //cachedT = new Pair<>(t1, t2);
            //tCachedFor = c;

            return new Pair<>(t1, t2);
        }
    }

    @Nullable
    // this does the same as previous, but also adds the constraint
    // that t in [0, 1]. If the circle has intersections but t is outside
    // the interval bounds for those, the function will return null
    public Pair<Double, Double> segmentIntersect_Circle(Circle2d c){
        Pair<Double, Double> res = intersect_Circle(c);
        if(res == null){
            return null;
        }

        Double t1 = res.getValue0();
        Double t2 = res.getValue1();
        if(t1 != null && 0 <= t1 && t1 <= 1){
            ;; // pass
        } else {
            t1 = null;
        }

        if(t2 != null && 0 <= t2 && t2 <= 1){
            ;; // pass
        } else {
            t2 = null;
        }

        if(t1 == null && t2 == null){
            return null;
        }

        return new Pair<>(t1, t2);
    }

    public double getSlope(){
        return slope;
    }
}
