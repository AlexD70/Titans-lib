package titans.geometry;

import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;

/**
 * A class for representing vectors in a 2d space.
 *
 *
 * <p>
 *     Provides both polar and cartesian representations. For performance reasons,
 *     the polar representation is optional and a constructor is provided specifically for this case.
 *     Contains various vector operations such as cross product, normalization, multiplying by scalars, etc.
 * </p>
 *
 * @author AlexD70
 */
public class Vector2d {
    private double x, y;
    private double r, t;
    private boolean isPolar = true;

    /**
     * Builds a vector with both cartesian and polar representation.
     */
    public Vector2d(double x, double y){
        this.x = x;
        this.y = y;
        this.toPolar();
    }

    /**
     * Builds a vector while allowing the user to choose whether a polar representation is needed.
     * If the vector is built with no polar representation, it can be added later through {@link #toPolar()}
     * at the cost of the future performance of the vector on certain operations.
     */
    public Vector2d(double x, double y, boolean noPolar){
        this.x = x;
        this.y = y;
        if(!noPolar){
            this.toPolar();
        }

        this.isPolar =  !noPolar;
    }

    /**
     * This static function provides a way to build a vector from polar coordinates.
     * @param r the absolute value
     * @param t the angle (in radians)
     * @return a new {@code Vector2d} object with both scalar and polar representations
     */
    public static Vector2d fromPolar(double r, double t){
        t = MathUtils.normalizeAngle(t, 0d);
        Vector2d ret = new Vector2d(FastMath.cos(t) * r, FastMath.sin(t) * r);
        ret.r = r;
        ret.t = t;

        return ret;
    }

    /**
     * Provides a way to use polar representation with a vector.
     * The user doesn't need to directly call this unless the vector was initialized with {@code noPolar = true}.
     */
    public void toPolar(){
        r = FastMath.sqrt(FastMath.pow(x, 2) + FastMath.pow(y, 2));
        t = MathUtils.normalizeAngle(FastMath.acos(x/r), 0d);
        this.isPolar = true;
    }

    /**
     * Rotates the vector with a certain angle. The object itself is modified.
     * @param deltaTheta the angle in radians
     * @return the object itself
     */
    public Vector2d rotate(double deltaTheta){
        this.toPolar();
        this.t += deltaTheta;
        this.t = MathUtils.normalizeAngle(this.t, 0d);

        this.x = FastMath.cos(this.t) * this.r;
        this.y = FastMath.sin(this.t) * this.r;
        return this;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    /**
     * @throws RuntimeException if no polar representation exists
     * @return the polar angle in radians between &pi; and -&pi;
     */
    public double getT(){
        if(!isPolar){
            throw new RuntimeException("Vector has no polar representation!");
        }
        return t;
    }

    /**
     * @throws RuntimeException if no polar representation exists
     * @return the absolute value
     */
    public double getR(){
        if(!isPolar){
            throw new RuntimeException("Vector has no polar representation!");
        }
        return r;
    }

    public double abs(){
        if(isPolar){
            return this.r;
        }
        return FastMath.sqrt(FastMath.pow(x, 2) + FastMath.pow(y, 2));
    }

    /**
     * The cross product of two 2d coplanar vectors results in a vector whose
     * only non-null component is the one orthogonal to the place, so it is represented
     * through a single double value.
     *
     */
    public double cross(Vector2d other){
        return this.x * other.y - this.y * other.x;
    }

    public Point2d toPoint(){
        return new Point2d(x, y);
    }

    /**
     * Multiplies the vector by a scalar. The vector is modified in this process.
     *
     * @return the object itself
     */
    public Vector2d mlt(double scalar){
        this.x *= scalar;
        this.y *= scalar;
        if(isPolar) {
            if(scalar < 0){
                this.t = MathUtils.normalizeAngle(this.t + FastMath.PI, 0d);
                scalar = -scalar;
            }
            this.r *= scalar;
        }
        return this;
    }

    /**
     * Two vectors are equal if their cartesian coordinates are both within {@code 1e-4} of each other.
     *
     * @return the result of the test mentioned
     */
    @Override
    public boolean equals(Object obj){
        if (obj instanceof Vector2d vect) {
            return (FastMath.abs(vect.getX() - this.x) < 0.0001) && (FastMath.abs(vect.getY() - this.y) < 0.0001);
        } else {
            return false;
        }
    }

    /**
     * Normalizes the vector by multiplying it with {@code 1/this.abs()}.
     * A vector with 0 absolute value is simply returned.
     * @return the object itself
     */
    public Vector2d norm() {
        double abs = this.abs();

        if(abs == 0){
            return this;
        }

        return this.mlt(1 / abs);
    }

    /**
     * @param other a {@code Vector2d}
     * @return a new {@code Vector2d} representing the difference
     */
    public Vector2d diff(Vector2d other){
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    /**
     * @param other a {@code Vector2d}
     * @return a new {@code Vector2d} representing the sum
     */
    public Vector2d add(Vector2d other){
        return new Vector2d(this.x + other.x, this.y + other.y);
    }
}
