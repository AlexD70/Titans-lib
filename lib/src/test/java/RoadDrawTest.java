import org.apache.commons.math3.util.Pair;
import org.json.JSONWriter;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import titans.geometry.Point2d;
import titans.roads.*;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class RoadDrawTest {
    @Tag("visual") @Test
    void showTestRoad() throws IOException, InterruptedException {
        RoadBuilder builder = new RoadBuilder(new Point2d(2, 2), 0, 0);
        Road road = builder
                .spline(new Point2d(30, 45))
                .line(new Point2d(30, 40))
                .spline(new Point2d(40, 30))
                .spline(new Point2d(50, 50))
                .build();

        drawRoad(road, 0.1);
    }

    @Tag("visual") @Test
    void showTestMotionProfile() throws IOException, InterruptedException {
        RoadBuilder builder = new RoadBuilder(new Point2d(), 0, 0);
        Road road = builder
                .spline(new Point2d(3, 3))
                .spline(new Point2d(10, 10))
                .line(new Point2d(4, 5))
                .line(new Point2d(4, 0)).build();
        ConstraintSet constr = new ConstraintSet(3.65, 2, 3.6, 1);
        drawRoad(road, 0.1);

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(road, constr, 0, 0, 0, 10, 10, 2000);
        drawProfile(profile);
    }

    void drawProfile(TrapezoidalMotionProfile profile) throws InterruptedException, IOException {
        StringBuffer jsonData = new StringBuffer();
        JSONWriter jsonifier = new JSONWriter(jsonData)
                .object()
                .key("points")
                .array();

        /*
        ArrayList<Pair<Double, KinematicState>> arr = profile.timeProfile;
        for(Pair<Double, KinematicState> p : arr){
            System.out.print("time ");
            System.out.println(p.getFirst());
            System.out.print("vel:");
            System.out.println(p.getSecond().velocity.abs());
            jsonifier.array().value(p.getFirst())
                    .value(p.getSecond().velocity.abs()).endArray();
        }

        jsonifier.endArray().endObject();
        System.out.print(jsonData);


        ProcessBuilder pythonProc = new ProcessBuilder(
                "python", new File("src/test/python/plot_road.py").getAbsolutePath(), jsonData.toString()
        );
        Process proc = pythonProc.start();
        proc.waitFor();

        assertEquals(0, proc.exitValue());
         */
    }

    // this test is for visual purposes... it doesnt actually check anything
    void drawRoad(Road road, double res) throws InterruptedException, IOException {
        StringBuffer jsonData = new StringBuffer();
        JSONWriter jsonifier = new JSONWriter(jsonData)
                .object()
                .key("points")
                .array();

        double disp = 0;
        while(disp <= road.getLength()){
            Point2d p = road.pointAtDisp(disp);
            jsonifier.array().value(p.getX()).value(p.getY()).endArray();
            disp += res;
        }

        jsonifier.endArray().endObject();
        System.out.print(jsonData);


        ProcessBuilder pythonProc = new ProcessBuilder(
                "python", new File("src/test/python/plot_road.py").getAbsolutePath(), jsonData.toString()
        );
        Process proc = pythonProc.start();
        proc.waitFor();

        assertEquals(0, proc.exitValue());
    }


}
