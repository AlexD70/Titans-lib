package titans.math;

import javax.annotation.Nonnull;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * Represents a polynomial of the n-th order. Provides a way to get the first and second derivatives directly.
 *
 * @author AlexD70
 */
public class NPoly {
    private final ArrayList<Double> coefficients = new ArrayList<>();
    private final int n;

    public NPoly(int degree){
        coefficients.ensureCapacity(degree + 1);
        n = degree;
    }

    /**
     * Coefficients are assigned from index 0 to the right from the highest power of x to the lowest as such:
     * <p>{@code coef[0] -> x^n}</p>
     * <p>{@code coef[1] -> x^(n - 1)}</p>
     * <p>...</p>
     * <p>{@code coef[n] -> free term (x^0)}</p>
     * <p>any excess coefficients are ignored</p>
     * If less than {@code n + 1} coefficients are provided, the rest of them are assumed to be 0.
     */
    public void assignCoefficients(@Nonnull double... coef){
        int i = 0;

        for(double c : coef){
            if(i > n){
                break;
            }
            coefficients.add(c);
            i ++;
        }

        for(; i <= n; i++){
            coefficients.add(0d);
        }
    }

    /**
     * Returns the value at {@code x} passed as a parameter.
     * @throws IndexOutOfBoundsException if the object's coefficients weren't assigned beforehand
     */
    public double apply(double x) throws IndexOutOfBoundsException {
        double xpowi = 1, res = 0;
        for(int i = 0; i <= n; i ++){
            res += coefficients.get(n - i) * xpowi;
            xpowi *= x;
        }

        return res;
    }

    /**
     * Computes the coefficients of the derivative.
     * @return a new {@code NPoly} object
     * @throws IndexOutOfBoundsException if the object's coefficients weren't assigned beforehand
     */
    @Nonnull
    public NPoly getDerivative() throws IndexOutOfBoundsException {
        NPoly deriv = new NPoly(n - 1);

        for(int i = 0; i < n; i ++){
            deriv.coefficients.add(coefficients.get(i) * (n - i));
        }

        return deriv;
    }

    @Nonnull
    public Double[] getCoefficients(){
        return coefficients.toArray(new Double[6]);
    }

    /**
     * Computes the coefficients of the second derivative.
     * @return a new {@code NPoly} object
     * @throws IndexOutOfBoundsException if the object's coefficients weren't assigned beforehand
     */
    @Nonnull
    public NPoly getSecondDerivative() throws IndexOutOfBoundsException {
        return getDerivative().getDerivative();
    }

    @Override
    public String toString() {
        return super.toString() + " coefficients: " + Arrays.toString(getCoefficients());
    }
}
