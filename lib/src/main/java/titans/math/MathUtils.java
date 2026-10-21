package titans.math;

import org.apache.commons.math3.util.FastMath;

public class MathUtils {
    public static final double DISP_EPS = 0.001;

    public static boolean dispLt(double disp, double operand2){
        return disp < operand2 + DISP_EPS;
    }

    public static boolean dispGt(double disp, double operand2){
        return disp > operand2 - DISP_EPS;
    }

    public static boolean dispInRange(double disp, double min, double max){
        return dispGt(disp, min) && dispLt(disp, max);
    }
}
