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

public class MathTests {
    public static final double EPS = 0.0001;
    public static void assertWithinError(double val, double ref){
        assertTrue(Math.abs(val - ref) <= EPS,
                String.format("Value %f out of the expected %f +- %f interval.", val, ref, EPS));
    }
    public static void assertInInterval(double val, double low, double high){
        assertTrue((low <= val) && (val <= high),
                String.format("Value %f out of interval [%f, %f]", val, low, high));
    }
    public static void assertWithinError(double val, double ref, double eps){
        assertTrue(Math.abs(val - ref) <= eps,
                String.format("Value %f out of the expected %f +- %f interval.", val, ref, eps));
    }

    void testDispFromParam(){
        Spline s = RoadResources.MathTests.testSpline;
        long millis = RoadResources.MathTests.MAX_MILLIS_INTEGRATION;

        double ufin = 1, urand = Math.random(), ufirst = urand / 100000d;

        ThrowingSupplier<Double> getLen = () -> s.dispAt(ufin);
        ThrowingSupplier<Double> getZero = () -> s.dispAt(ufirst);
        ThrowingSupplier<Double> getAny = () -> s.dispAt(urand);
        double resLen = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getLen);
        double resZero = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getZero);
        double resRand = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getAny);

        assertWithinError(resLen, s.getLength());
        assertWithinError(resZero, 0);
        assertTrue(0 <= resRand && resRand <= s.getLength(),
                String.format("ResRand %f exceeds bounds of 0, %f when u = %f.", resRand, s.getLength(), urand));
    }

    void testSplineEndPoint(){
        Spline s = RoadResources.MathTests.testSpline;

        Point2d endp = RoadResources.MathTests.points[1];

        RoadBuilderTests.assertPointsEqual(endp, s.pointAt(1));
        RoadBuilderTests.assertPointsEqual(endp, s.pointAtDisp(s.getLength()));
    }

    void testParamFromDisp(){
        Spline s = RoadResources.MathTests.testSpline;
        long millis = RoadResources.MathTests.MAX_MILLIS_BRENTOPT;

        double len = s.getLength(), rand = Math.random() * len;
        ThrowingSupplier<Double> getOne = () -> s.uAtDisp(len);
        ThrowingSupplier<Double> getZero = () -> s.uAtDisp(0);
        ThrowingSupplier<Double> getAny = () -> s.uAtDisp(rand);
        double one = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getOne);
        double zero = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getZero);
        double any = assertTimeout(Duration.of(millis, ChronoUnit.MILLIS), getAny);

        assertWithinError(one, 1);
        assertWithinError(zero, 0);
        assertTrue(0 <= any && any <= 1,
                String.format("URand %f out of [0, 1] expected interval for internal param.", any));
    }

    void testDerivatives(){
        Spline s = RoadResources.MathTests.testSpline;

        double rand = Math.random();
        Point2d ds = s.derivAt(rand), dds = s.deriv2ndAt(rand);

        assertEquals(ds, s.derivAt(rand));
        assertEquals(dds, s.deriv2ndAt(rand));
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
        testDerivatives();
        testTangents();
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
