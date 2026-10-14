package titans.math;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.apache.commons.math3.util.FastMath;

import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import titans.util.NullSplineErr;
import titans.util.OutOfBoundsException;

import javax.annotation.Nonnull;
import javax.annotation.ParametersAreNonnullByDefault;

/**
 * A class representing splines which are a type of parametric curves
 * using two {@code NPoly} objects to compute the cartesian coordinates from
 * the parameter {@code u} from the interval [0, 1].
 *
 * <p>
 *     Provides several mathematical functions, displacement reparametrization, derivatives and tangents.
 *     Most of the class' functions will throw {@code NullSplineErr} if the user initializes a "null"
 *     {@code Spline} and does not assign its polynomials accordingly before doing operations with it.
 * </p>
 *
 * @see NPoly
 * @author AlexD70
 */
@ParametersAreNonnullByDefault
public class Spline implements ParametricCurve {
    private NPoly xpoly = null, ypoly = null;
    private NPoly xderiv = null, yderiv = null;
    private NPoly x2ndderiv = null, y2ndderiv = null;

    private double length = -1;
    private boolean isNull = true;

    private Point2d startPoint = null, endPoint = null;

    // for Brent optimizer caching
    private double initialGuessCache = 0.5;

    private static final double[][] invertedSystemMatrixRaw = {
            {-6,  -3, -0.5,  6,  -3,  0.5},
            { 15,  8,  1.5, -15,  7, -1},
            {-10, -6, -1.5,  10, -4,  0.5},
            { 0,   0,  0.5,  0,   0,  0},
            { 0,   1,  0,    0,   0,  0},
            { 1,   0,  0,    0,   0,  0}
    };
    private static final RealMatrix invertedSystemMatrix = new BlockRealMatrix(invertedSystemMatrixRaw);

    /**
     * Builds a new "null" {@code Spline}. The intended use is for the user to {@link #setSpline(NPoly, NPoly, Point2d)}
     * later so that the {@code isNull} flag clears and the object starts working as intended.
     */
    public Spline(){}

    /**
     * Builds a new fully-initialized {@code Spline} object with the flag {@code isNull} set to
     * {@code false}. The function interpolates the input using quintic polynomials.
     * @param start the start point
     * @param end the end point
     * @param startDeriv the first derivative on both x and y at the start point
     * @param endDeriv the first derivative on both x and y at the end point
     * @param start2ndDeriv the second derivative on both x and y at the start point
     * @param end2ndDeriv the second derivative on both x and y at the end point
     */
    public Spline(
            Point2d start, Point2d end,
            Point2d startDeriv, Point2d endDeriv,
            Point2d start2ndDeriv, Point2d end2ndDeriv
    ){
        double[][] rValueX = {
                {start.getX()}, {startDeriv.getX()}, {start2ndDeriv.getX()},
                {end.getX()}, {endDeriv.getX()}, {end2ndDeriv.getX()}
        };
        double[][] rValueY = {
                {start.getY()}, {startDeriv.getY()}, {start2ndDeriv.getY()},
                {end.getY()}, {endDeriv.getY()}, {end2ndDeriv.getY()}
        };

        RealMatrix resultX = invertedSystemMatrix.multiply(new BlockRealMatrix(rValueX));
        RealMatrix resultY = invertedSystemMatrix.multiply(new BlockRealMatrix(rValueY));

        NPoly xpoly = new NPoly(5);
        xpoly.assignCoefficients(resultX.getColumn(0));
        NPoly ypoly = new NPoly(5);
        ypoly.assignCoefficients(resultY.getColumn(0));

        this.setSpline(xpoly, ypoly, start);

        this.endPoint = end;
        this.unsetNull();
    }

    // INITIALIZING ======================================

    /**
     * This function attempts clearing the {@code isNull} flag from the object. To successfully do so,
     * the user must first use the {@link #setSpline(NPoly, NPoly, Point2d)} function on the object. On fail,
     * the function will throw a {@code RuntimeException}. The spline {@code length} and {@code endPoint} will
     * automatically be computed from the polynomials given.
     * @throws RuntimeException on fail <3
     */
    public void unsetNull() {
        if(!isNull){
            return;
        }
        if(xpoly != null && ypoly != null){
            isNull = false;
            this.xderiv = xpoly.getDerivative();
            this.x2ndderiv = xpoly.getSecondDerivative();
            this.yderiv = ypoly.getDerivative();
            this.y2ndderiv = ypoly.getSecondDerivative();
            this.length = dispAt(1);
            if(endPoint == null){
                endPoint = pointAt(1);
            }
        } else {
            throw new RuntimeException("Cannot unset null with empty polynomials!");
        }
    }

    /**
     * Initializes a spline that was created using {@link #Spline()}.
     * @param xpoly a polynomial x(u), u in [0, 1]
     * @param ypoly a polynomial y(u), u in [0, 1]
     * @param startPoint the start point x(0), y(0)
     * @throws RuntimeException if the spline was created using {@link #Spline(Point2d, Point2d, Point2d, Point2d, Point2d, Point2d)}
     * or if it was previously initialized with this function
     */
    public void setSpline(NPoly xpoly, NPoly ypoly, Point2d startPoint) {
        if(!isNull){
            throw new RuntimeException("Do not modify non-null splines!");
        }

        this.xpoly = xpoly;
        this.ypoly = ypoly;
        this.startPoint = startPoint;
        unsetNull();
    }

    // ===================================================
    // GETTERS ===========================================

    @Override
    @Nonnull
    public Point2d getStartPoint(){
        if (isNull) {
            throw new NullSplineErr();
        }
        return startPoint;
    }

    @Override
    @Nonnull
    public Point2d getEndPoint(){
        if (isNull) {
            throw new NullSplineErr();
        }
        return endPoint;
    }

    @Override
    public double getLength(){
        return length;
    }

    @Nonnull
    public Double[] getXCoeffs(){
        if (isNull) {
            throw new NullSplineErr();
        }
        return xpoly.getCoefficients();
    }

    @Nonnull
    public Double[] getYCoeffs(){
        if (isNull) {
            throw new NullSplineErr();
        }
        return ypoly.getCoefficients();
    }

    // =====================================================
    // CALCULATIONS WITH INTERNAL PARAM ====================

    /**
     * Applies the spline at a given parameter.
     * @param u the internal parameter; between 0 and 1
     * @return a new {@code Point2d} object
     * @throws NullSplineErr
     */
    @Override @Nonnull
    public Point2d pointAt(double u){
        if(isNull) {
            throw new NullSplineErr();
        }
        if(u == 0){
            return startPoint;
        } else if (u == 1){
            return endPoint;
        }

        return new Point2d(xpoly.apply(u), ypoly.apply(u));
    }

    /**
     * Integrates the position vector over (0, u) to get the distance travelled.
     * @param u the internal parameter; between 0 and 1
     * @return distance travelled until the given parameter
     * @throws NullSplineErr see class description
     * @throws TooManyEvaluationsException
     */
    @Override
    public double dispAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }
        if(u == 0){
            return 0;
        } else if (u == 1){
            return length;
        }

        SimpsonIntegrator integrator = new SimpsonIntegrator();
        try {
            return integrator.integrate(
                    1000,
                    (t) -> FastMath.sqrt(FastMath.pow(xderiv.apply(t), 2) + FastMath.pow(yderiv.apply(t), 2)),
                    0,
                    u
            );
        } catch (TooManyEvaluationsException e) {
            try {
                return integrator.integrate(
                        1_000_000,
                        (t) -> FastMath.sqrt(FastMath.pow(xderiv.apply(t), 2) + FastMath.pow(yderiv.apply(t), 2)),
                        0,
                        u
                );
            } catch (TooManyEvaluationsException e2) {
                // TODO: this shouldnt crash the whole program - it might be better to throw a new error or log the error
                e.printStackTrace();
                System.exit(-1);
            }
        }

        return 0;
    }

    @Override
    @Nonnull
    public Point2d derivAt(double u) {
        if(isNull){
            throw new NullSplineErr();
        }
        return new Point2d(xderiv.apply(u), yderiv.apply(u));
    }

    @Override
    @Nonnull
    public Point2d deriv2ndAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }
        return new Point2d(
                x2ndderiv.apply(u),
                y2ndderiv.apply(u)
        );
    }

    /**
     * Computes the curve slope at a given internal parameter.
     * @param u the internal parameter (in [0, 1])
     * @return tangent angle in radians
     * @throws NullSplineErr
     */
    @Override
    public double tanAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }
        Vector2d deriv = derivAt(u).toVector();
        return deriv.getT();
    }

    /**
     * Computes the tangent vector at a given internal parameter. The vector is normal
     * and has both cartesian and polar representations.
     * @param u the internal parameter (in [0, 1])
     * @return a new {@code Vector2d} object
     * @throws NullSplineErr
     */
    @Override
    @Nonnull
    public Vector2d tanVecAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }
        return derivAt(u).toVector().norm();
    }

    // ========================================================
    // CALCULATIONS WITH DISPLACEMENT =========================

    /**
     * Computes the internal parameter at a given displacement.
     * For better performance, the function caches the last result so that the solver gets at the solution faster.
     * @param d displacement in cm
     * @return internal parameter between [0, 1]
     * @throws NullSplineErr see class description
     * @throws OutOfBoundsException if displacement is negative or greater than {@code length}
     */
    @Override
    public double uAtDisp(double d){
        if(isNull){
            throw new NullSplineErr();
        }
        if(length != -1 && (d > length || d < 0)){
            throw new OutOfBoundsException();
        }

        var fx = new UnivariateFunction() {
            @Override
            public double value(double x) {
                // do the absolute value because brentopt
                // doesnt search for fx = 0 but rather searches the function minimum
                // and if the function happens to be negative at any point
                // then the brentopt returns that rather than the desired point
                return FastMath.abs(dispAt(x) - d);
            }
        };

        var brentopt = new BrentOptimizer(1e-6, 1e-7);
        UnivariatePointValuePair res = brentopt.optimize(
                new MaxEval(Integer.MAX_VALUE),
                new MaxIter(1000),
                new UnivariateObjectiveFunction(fx),
                GoalType.MINIMIZE, new SearchInterval(0, 1),
                // not sure this is a good first guess - some benchmarking required
                new InitialGuess(new double[]{0, initialGuessCache, 1})
        );

        initialGuessCache = res.getPoint();
        return initialGuessCache;
    }

    /**
     * @throws NullSplineErr see class description
     * @throws OutOfBoundsException see {@link #uAtDisp(double)}
     */
    @Override
    @Nonnull
    public Point2d pointAtDisp(double d){
        if(isNull){
            throw new NullSplineErr();
        }
        if(d == 0){
            return startPoint;
        }
        return pointAt(uAtDisp(d));
    }

}