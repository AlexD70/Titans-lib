import org.apache.commons.math3.util.FastMath;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.roads.Road;
import titans.roads.RoadBuilder;

public class HeadingInterpTest {
    @Test @Tag("test")
    void testLineToLinearHeading(){
        RoadBuilder builder = new RoadBuilder(new Point2d(), 0, 0, 0);

        Road r = builder.lineLinearHeading(new Point2d(10, 10), FastMath.PI / 2).build();

        Vector2d startHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0);
        Vector2d midHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0.5);
        Vector2d endHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(1);
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", startHeading.getX(), startHeading.getY(), startHeading.getR(), startHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", midHeading.getX(), midHeading.getY(), midHeading.getR(), midHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", endHeading.getX(), endHeading.getY(), endHeading.getR(), endHeading.getT());
    }

    @Test @Tag("test")
    void testLineToSplineHeading(){
        RoadBuilder builder = new RoadBuilder(new Point2d(), 0, 0, 0);

        Road r = builder.lineSplineHeading(new Point2d(10, 10), FastMath.PI / 2, FastMath.PI / 3).build();

        Vector2d startHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0);
        Vector2d midHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0.5);
        Vector2d endHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(1);
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", startHeading.getX(), startHeading.getY(), startHeading.getR(), startHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", midHeading.getX(), midHeading.getY(), midHeading.getR(), midHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", endHeading.getX(), endHeading.getY(), endHeading.getR(), endHeading.getT());
    }

    @Test @Tag("test")
    void testSplineToLinearHeading(){
        RoadBuilder builder = new RoadBuilder(new Point2d(), 0, 0, 0);

        Road r = builder.splineLinearHeading(new Point2d(10, 10), FastMath.PI / 2).build();

        Vector2d startHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0);
        Vector2d midHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0.5);
        Vector2d endHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(1);
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", startHeading.getX(), startHeading.getY(), startHeading.getR(), startHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", midHeading.getX(), midHeading.getY(), midHeading.getR(), midHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", endHeading.getX(), endHeading.getY(), endHeading.getR(), endHeading.getT());
    }

    @Test @Tag("test")
    void testSplineToSplineHeading(){
        RoadBuilder builder = new RoadBuilder(new Point2d(), 0, 0, 0);

        Road r = builder.splineSplineHeading(new Point2d(10, 10), FastMath.PI / 2, FastMath.PI, FastMath.PI / 2).build();

        Vector2d startHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0);
        Vector2d midHeading = r.getHeadingInterpByIndex(0).getNormedVectorAtParam(0.5);
        Vector2d endHeading = r.getHeadingInterpByIndex(0).getVectorAtParam(1);
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", startHeading.getX(), startHeading.getY(), startHeading.getR(), startHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", midHeading.getX(), midHeading.getY(), midHeading.getR(), midHeading.getT());
        System.out.printf("(%f, %f) -> (r:%f, t:%f)%n", endHeading.getX(), endHeading.getY(), endHeading.getR(), endHeading.getT());
    }
}
