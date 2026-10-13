package titans.math;

import org.apache.commons.math3.util.FastMath;

public class HermiteInterpolation {
    private double x0, x1;
    private double m0, m1;
    private double coef3, coef2, coef1, coef0;

    public HermiteInterpolation(double x0, double x1, double m0, double m1) {
        coef3 = 2 * x0 - 2 * x1 + m0 + m1;
        coef2 = -3 * x0 + 3 * x1 - 2 * m0 - m1;
        coef1 = m0;
        coef0 = x0;
    }

    public double apply(double t){
        double t2 = FastMath.pow(t, 2);
        double t3 = FastMath.pow(t, 3);

        return t3 * coef3 + t2 * coef2 + t * coef1 + coef0;
    }

    public double applyDerivative(double t){
        return 3 * FastMath.pow(t, 2) * coef3 + 2 * t * coef2 + coef1;
    }
}
