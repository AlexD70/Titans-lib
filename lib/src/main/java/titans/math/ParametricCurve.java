package titans.math;

import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import javax.annotation.Nonnull;

public interface ParametricCurve {
    @Nonnull Point2d getStartPoint();
    @Nonnull Point2d getEndPoint();
    double getLength();
    @Nonnull Point2d pointAt(double u);
    double dispAt(double u);
    @Nonnull Point2d derivAt(double u);
    @Nonnull Point2d deriv2ndAt(double u);
    double tanAt(double u);
    @Nonnull Vector2d tanVecAt(double u);
    double uAtDisp(double d);
    @Nonnull Point2d pointAtDisp(double d);
}
