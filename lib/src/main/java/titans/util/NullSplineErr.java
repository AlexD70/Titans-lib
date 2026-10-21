package titans.util;

import titans.math.Spline;

import java.util.Objects;

public class NullSplineErr extends RuntimeException {
    public NullSplineErr(Spline s){
        super(String.format("thrown by %s involving object %s.", getCallerFunction(), s.toString()));
    }

    static private String getCallerFunction(){
        return StackWalker.getInstance()
                .walk(
                        frame -> frame.filter(
                                _frame -> Objects.equals(_frame.getClassName(), Spline.class.getName()
                                )
                        ).findFirst()
                ).map(__frame -> __frame.getClassName() + "." + __frame.getMethodName())
                .orElse("Unknown caller!");
    }
}
