package titans.util;

public class DispOfBoundsException extends RuntimeException {
    public DispOfBoundsException(double erroneousDisplacement, double min, double max, Object cause){
        super(String.format("displacement %f out of range [%f, %f]. Cause object: ", erroneousDisplacement, min, max) + cause);
    }
}
