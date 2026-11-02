import org.apache.commons.math3.util.FastMath;
import org.javatuples.Triplet;
import org.json.JSONWriter;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.function.ThrowingSupplier;
import titans.math.NPoly;
import titans.geometry.Point2d;
import titans.math.Spline;
import titans.util.NullSplineErr;

import java.io.File;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.Scanner;

import static org.junit.jupiter.api.Assertions.*;


public class SplineTests {

    @Tag("test") @Test
    void testDispFromParam(){
        Spline s = TestResources.getRandomSpline();
        long millis = TestResources.MAX_MILLIS_INTEGRATION;

        double ufin = 1, urand = Math.random(), ufirst = urand / 100000d;

        ThrowingSupplier<Double> getLen = () -> s.dispAt(ufin);
        ThrowingSupplier<Double> getZero = () -> s.dispAt(ufirst);
        ThrowingSupplier<Double> getAny = () -> s.dispAt(urand);
        double resLen = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getLen);
        double resZero = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getZero);
        double resRand = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getAny);

        TestResources.assertWithinError(resLen, s.getLength());
        TestResources.assertWithinError(resZero, 0);
        assertTrue(0 <= resRand && resRand <= s.getLength(),
                String.format("ResRand %f exceeds bounds of 0, %f when u = %f.", resRand, s.getLength(), urand));
    }

    @Tag("test") @Test
    void testSplineEndPoint(){
        Triplet<Spline, Point2d, Point2d> testRes = TestResources.getRandomSplineAndEndpoints();
        Spline s = testRes.getValue0();

        Point2d start = testRes.getValue1();
        Point2d end = testRes.getValue2();

        TestResources.assertPointsEqual(start, s.pointAt(0));
        TestResources.assertPointsEqual(start, s.pointAtDisp(0));
        TestResources.assertPointsEqual(start, s.getStartPoint());

        TestResources.assertPointsEqual(end, s.pointAt(1));
        TestResources.assertPointsEqual(end, s.pointAtDisp(s.getLength()));
        TestResources.assertPointsEqual(end, s.getEndPoint());
    }

    @Tag("test") @Test
    void testParamFromDisp(){
        Spline s = TestResources.getRandomSpline();
        long millis = TestResources.MAX_MILLIS_BRENTOPT;

        double len = s.getLength(), rand = Math.random() * len;
        ThrowingSupplier<Double> getOne = () -> s.uAtDisp(len);
        ThrowingSupplier<Double> getZero = () -> s.uAtDisp(0);
        ThrowingSupplier<Double> getAny = () -> s.uAtDisp(rand);
        double one = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getOne);
        double zero = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getZero);
        double any = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getAny);

        TestResources.assertWithinError(one, 1);
        TestResources.assertWithinError(zero, 0);
        assertTrue(0 <= any && any <= 1,
                String.format("URand %f out of [0, 1] expected interval for internal param.", any));
    }

    @Tag("test") @Test @Disabled
    void testDerivatives(){
        Spline s = RoadResources.MathTests.testSpline;

        double ds_x = 0, dds_x = 0, ds_y = 0, dds_y = 0;
        Double[] xcoeffs = s.getXCoeffs(), ycoeffs = s.getYCoeffs();
        double rand = FastMath.random();
        double randToNth = 1; // ma cac pe ele de derivate
        for (int i = xcoeffs.length - 1; i >= 0; i--){
            ds_x += randToNth * xcoeffs[i] * (xcoeffs.length - i - 1) * rand;
            dds_x += randToNth * xcoeffs[i] * (xcoeffs.length - i - 2) * (xcoeffs.length - i - 1) / rand;
            if(i <= xcoeffs.length - 2) {
                randToNth *= rand;
            }
        }
        randToNth = 1;
        for (int i = ycoeffs.length - 1; i >= 0; i--){
            ds_y += randToNth * ycoeffs[i] * (ycoeffs.length - i);
            dds_y += randToNth * ycoeffs[i] * (ycoeffs.length - i - 1) * (ycoeffs.length - i) / rand;
            randToNth *= rand;
        }

        assertEquals(new Point2d(ds_x, ds_y), s.derivAt(rand));
        assertEquals(new Point2d(dds_x, dds_y), s.deriv2ndAt(rand));
    }

    void testTangents(){
        Spline s = RoadResources.MathTests.testSpline;
        Spline s2 = RoadResources.MathTests.testSpline2;

        double rand = Math.random();
        double tangent1 = s.tanAt(rand);
        double tangent2 = s2.tanAt(rand);
        //assertInInterval(tangent1, -Math.PI, Math.PI);
        assertEquals(tangent1, s.tanAt(rand));
        //assertInInterval(tangent2, -Math.PI, Math.PI);
        assertEquals(tangent2, s2.tanAt(rand));
    }

    @Tag("test") @Test
    void testNullSpline(){
        Spline s = new Spline();
        assertThrowsExactly(NullSplineErr.class, () -> s.tanAt(1)).printStackTrace();
        assertThrowsExactly(NullSplineErr.class, () -> s.uAtDisp(0)).printStackTrace();
        assertThrowsExactly(NullSplineErr.class, () -> s.derivAt(0)).printStackTrace();
        assertThrowsExactly(NullSplineErr.class, () -> s.dispAt(0)).printStackTrace();
        assertThrowsExactly(NullSplineErr.class, () -> s.pointAtDisp(0)).printStackTrace();
        assertThrowsExactly(NullSplineErr.class, () -> s.deriv2ndAt(0)).printStackTrace();
        assertThrowsExactly(NullSplineErr.class, s::getXCoeffs).printStackTrace();
        assertThrowsExactly(NullSplineErr.class, s::getYCoeffs).printStackTrace();

        assertThrows(IllegalCallerException.class, s::unsetNull).printStackTrace();
        NPoly xpoly = new NPoly(5);
        xpoly.assignCoefficients(1, 1, 1, 1, 1, 1);
        NPoly ypoly = new NPoly(5);
        ypoly.assignCoefficients(0, 0, 1, 0, 1, 0);
        assertDoesNotThrow(() -> s.setSpline(xpoly, ypoly, new Point2d(0, 0)));
        assertThrows(IllegalCallerException.class, () -> s.setSpline(xpoly, ypoly, new Point2d(0, 0))).printStackTrace();
    }

    @Tag("test") @Test
    void testMathematicalStability() {
        testDispFromParam();
        testParamFromDisp();
        //testDerivatives();
        //testTangents();
        testNullSpline();
        testSplineEndPoint();
    }


    // this test checks if the python output and the java output
    // are within an error margin of 0.00001 (set in the py source)
    // also checks that buildSpline6 runs in an acceptable
    // time of 30 ms (set in the py source)
    @Tag("visual") @Test
    void testSplineBuild() throws IOException, InterruptedException {
        // get java output
        Instant start = Instant.now();
        Point2d[] p = RoadResources.MathTests.points;
        Spline s = new Spline(p[0], p[3], p[1], p[4], p[2], p[5]);
        Instant end = Instant.now();

        long elapsed = Duration.between(start, end).toMillis();

        // encode data to json
        StringBuffer jsonData = new StringBuffer();
        JSONWriter jsonifier = new JSONWriter(jsonData)
                .object()
                .key("points")
                .array();

        for(Point2d _p : p){
            jsonifier.array().value(_p.getX()).value(_p.getY()).endArray();
        }

        Double[] xcoeffs = s.getXCoeffs(), ycoeffs = s.getYCoeffs();
        jsonifier.endArray()
                .key("xcoef")
                .value(xcoeffs)
                .key("ycoef")
                .value(ycoeffs)
                .key("time")
                .value(elapsed)
                .endObject();

        System.out.print(jsonData);

        // call python and pass json data as sys.argv[1]
        ProcessBuilder pythonProc = new ProcessBuilder(
                "python", new File("src/test/python/path_solve_test.py").getAbsolutePath(), jsonData.toString()
        );
        Process proc = pythonProc.start();
        Scanner reader = new Scanner(proc.getInputStream());

        // print python output
        System.out.print("\n");
        while(reader.hasNextLine()) {
            System.out.println(reader.nextLine());
        }

        // wait for exit code and assert its value is 0
        proc.waitFor();
        assertEquals(0, proc.exitValue());
    }
}
