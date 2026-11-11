package titans.util;

/**
 * A runtime exception to be thrown when a displacement or any bounded double value in the program fails a loose check of the type
 * {@code min + EPS < val < max + EPS} and even if the result of following computations would exist,
 * the result would not make sense or corrupt other calculations.
 *
 * @author AlexD70
 */
public class DispOfBoundsException extends RuntimeException {

    /**
     * The constructor of the exception class.
     * @param erroneousDisplacement the value that failed the loose check
     * @param min the minimum value, before subtracting EPS
     * @param max the maximum value, before adding EPS
     * @param cause the object that threw the error itself. If no object threw the error, pass {@code new Object()} (never pass null)
     */
    public DispOfBoundsException(double erroneousDisplacement, double min, double max, Object cause){
        super(String.format("displacement %f out of range [%f, %f]. Cause object: ", erroneousDisplacement, min, max) + cause);
    }
}
