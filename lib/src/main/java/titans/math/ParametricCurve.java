package titans.math;

import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import javax.annotation.Nonnull;

/**
 * The ParametricCurve public interface provides the required functions that a parametric curve needs
 * for the program to work. All parameters and return types are annotated with {@code @Nonnull} as required since
 * the callers usually require that what they get is not null. Implementations of the given interface should faithfully
 * respect these annotations.
 *
 * <p>
 * This represents a 2D curve that is parameterized between 0 and 1 (the internal parameter referred to as "u" should be
 * in the interval [0, 1]).
 * </p>
 *
 * @author AlexD70
 */
public interface ParametricCurve {

    /**
     * This function should return the start point of the parametric curve. That is, the point on the curve
     * at internal parameter u = 0. This should be equivalent to {@code pointAt(0)} in the sense that the returned points
     * are close enough to each other so that the expression {@code pointAt(0).equals(getStartPoint())} evaluates to true.
     *
     * @return the start point of the curve
     * @see Point2d
     */
    @Nonnull Point2d getStartPoint();

    /**
     * This function should return the end point of the parametric curve. That is, the point on the curve
     * at internal parameter u = 1. This should be equivalent to {@code pointAt(1)} in the sense that the returned points
     * are close enough to each other so that the expression {@code pointAt(1).equals(getEndPoint())} evaluates to true.
     *
     * @return the end point of the curve
     * @see Point2d
     */
    @Nonnull Point2d getEndPoint();

    /**
     * Returns the curve length as a double.
     * @return curve length
     */
    double getLength();

    /**
     * Returns the point on the curve at an arbitrary internal parameter.
     * @param u the internal parameter in [0, 1]. The implementing class should check if the argument is indeed in the right range value, or throw {@link titans.util.DispOfBoundsException}
     * @return the required point
     */
    @Nonnull Point2d pointAt(double u);

    /**
     * Returns the displacement on the curve at a given internal parameter. Generally,
     * the displacement is the sum of small Euclidean distances from close points from the start point to
     * the point at u.
     * @param u the internal parameter in [0, 1]. The implementing class should check if the argument is indeed in the right range value
     *          or throw {@link titans.util.DispOfBoundsException}
     * @return a valid displacement as a double (positive and smaller or equal to {@code getLength()})
     */
    double dispAt(double u);


    @Nonnull Point2d derivAt(double u);
    @Nonnull Point2d deriv2ndAt(double u);
    double tanAt(double u);
    @Nonnull Vector2d tanVecAt(double u);
    double uAtDisp(double d);
    @Nonnull Point2d pointAtDisp(double d);
}
