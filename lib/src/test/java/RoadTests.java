import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import titans.math.NPoly;
import titans.geometry.Point2d;
import titans.math.ParametricCurve;
import titans.roads.Road;
import titans.math.Spline;

import java.util.Arrays;

import org.apache.commons.math3.util.Pair;

import static org.junit.jupiter.api.Assertions.*;

public class RoadTests {
    static NPoly c1x = new NPoly(5), c1y = new NPoly(5);
    static NPoly c2x = new NPoly(5), c2y = new NPoly(5);
    static {
        c1x.assignCoefficients(0, 0, 0, 0, 1, -2.3);
        c1y.assignCoefficients(0, 0, 0, -4, 1, 4);
        c2x.assignCoefficients(1.2, 0.1, 3, 1, -1, -1.3);
        c2y.assignCoefficients(0.5, 4, -1, 4, 0.2, 1);
    }
    static final double lenc1 = 3.4023;
    static final double MAX_LEN_ERR = 0.01;
    public Road road = new Road();

    public void buildTestRoad() {
        Spline c1 = new Spline();
        Spline c2 = new Spline();

        assertDoesNotThrow(() -> c1.setSpline(c1x, c1y, new Point2d(c1x.apply(0), c1y.apply(0))));
        assertDoesNotThrow(() -> c2.setSpline(c2x, c2y, new Point2d(c2x.apply(0), c2y.apply(0))));

        road.addSegment(c1, false);
        road.addSegment(c2, false);
    }

    @Tag("test") @Test
    void checkCurveLen(){
        buildTestRoad();

        System.out.println(road.getSegment2Disp(0).getFirst().dispAt(1));

        Pair<ParametricCurve, Double> output = road.getSegment2Disp(2);
        Spline spline;

        if(output.getFirst() instanceof Spline){
            spline = (Spline) output.getFirst();
        } else {
            assert(false);
            return;
        }

        System.out.println(Arrays.toString(spline.getXCoeffs()));
        System.out.println(Arrays.toString(spline.getYCoeffs()));
        System.out.println(output.getSecond());

        Point2d point = road.pointAtDisp(1);
        System.out.printf("(%f, %f)%n", point.getX(), point.getY());
    }
}
