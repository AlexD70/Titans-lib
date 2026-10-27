package titans.roads;

import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.math.MathUtils;
import titans.math.ParametricCurve;
import titans.math.Spline;
import titans.util.*;

import org.javatuples.Triplet;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.*;

/**
 * This class represents a larger pathing unit comprised of several <b>ordered</b> {@link Spline}s and {@link HeadingInterpolation}s that
 * ideally should be bound together through start and endpoints (this is why the {@link RoadBuilder} exists).
 * Do not use build objects of this class directly unless you know what you are doing.
 *
 * <p>
 *     This class provides a plethora of functions that look up through all splines and heading interpolations
 *     to return the required information which can be searched by index or road displacement. Note that road displacement
 *     is not always the same as spline displacement as the displacement on each spline starts from 0 while the road
 *     accumulates the length of all of its components.
 * </p>
 * <p>
 *     The class has a hardcoded segment limit: {@code MAX_SEGMENTS = 30}. The segments should be added using either
 *     {@link #addSegment(ParametricCurve, boolean)} or {@link #addSegment(ParametricCurve, boolean, HeadingInterpolation)}.
 *     After adding, a segment cannot be removed or otherwise modified unless the class is inherited from.
 * </p>
 *
 * @author AlexD70
 * @see Spline
 * @see RoadBuilder
 * @see HeadingInterpolation
 */
public class Road {
    private final static int MAX_SEGMENTS = 30;

    protected ArrayList<ParametricCurve> segments = new ArrayList<>(MAX_SEGMENTS + 1);
    protected ArrayList<Boolean> isContinous = new ArrayList<>(MAX_SEGMENTS + 1);
    protected ArrayList<HeadingInterpolation> headingInterp = new ArrayList<>(MAX_SEGMENTS + 1);
    protected double[] lenarr = new double[MAX_SEGMENTS + 1];

    private double totalLength = 0;
    private int n = 0;
    private double eps = 0.005;

    /**
     * Builds a new {@code Road} object with an empty segment list.
     * Do not use unless you know what you are doing.
     */
    public Road(){}

    /**
     * Adds a new segment to the road comprised of a {@code Spline}, a boolean telling the
     * class whether the new spline has the same tangent vector as the previous and a new, empty {@code HeaingInterpolation}.
     * @param s a non-{@code isNull} flagged {@code Spline} object
     * @param continuous a boolean showing if path continuity is broken or not
     * @throws RuntimeException if segment count exceeds {@code MAX_SEGMENTS}
     */
    public void addSegment(ParametricCurve s, boolean continuous){
        if(n == MAX_SEGMENTS){
            throw new RuntimeException("Max segment count exceeded for path!");
        }

        segments.add(s);
        isContinous.add(continuous);
        headingInterp.add(new HeadingInterpolation());
        totalLength += s.getLength();
        lenarr[n] = totalLength;
        n += 1;
    }

    /**
     * Adds a new segment to the road comprised of a {@code Spline}, a boolean telling the
     * class whether the new spline has the same tangent vector as the previous and a {@code HeaingInterpolation}.
     * @param s a non-{@code isNull} flagged {@code Spline} object
     * @param continuous a boolean showing if path continuity is broken or not
     * @param interp a heading interpolation object
     * @throws RuntimeException if segment count exceeds {@code MAX_SEGMENTS}
     */
    public void addSegment(ParametricCurve s, boolean continuous, HeadingInterpolation interp) {
        if(n == MAX_SEGMENTS){
            throw new RuntimeException("Max segment count exceeded for path!");
        }

        segments.add(s);
        isContinous.add(continuous);
        headingInterp.add(interp);
        totalLength += s.getLength();
        lenarr[n] = totalLength;
        n += 1;
    }

    public HeadingInterpolation getHeadingInterpByIndex(int idx){
        return headingInterp.get(idx);
    }

    // internal helper
    private int binSearchRecursive(int start, int end, double search){
        if(search <= lenarr[start]){
            return start;
        }

        if(start == end - 1){
            return end;
        }

        if(lenarr[(start + end) / 2] >= search){
            return binSearchRecursive(start, (start + end) / 2, search);
        } else {
            return binSearchRecursive((start + end) / 2, end, search);
        }
    }

    /**
     * Searches the correct segment by road displacement.
     * @param d displacement in cm
     * @return a {@code Pair<Spline, Double>} containing the segment as a {@code Spline} and the relative displacement on that segment
     * @throws RuntimeException if the displacement is out of road bounds
     */
    public Pair<ParametricCurve, Double> getSegment2Disp(double d){
        if(!MathUtils.dispInRange(d, 0, totalLength)){
            throw new DispOfBoundsException(d, 0, totalLength, this);
        }

        int index = binSearchRecursive(0, n, d);
        return new Pair<>(segments.get(index), FastMath.abs(d - ((index == 0)?(0):(lenarr[index - 1]))));
    }

    /**
     * Searches the correct segment by road displacement.
     * @param d displacement in cm
     * @return a {@code Triplet<Spline, Integer, Double>} containing the segment as a {@code Spline}, the segment index and
     * the relative displacement on that segment
     * @throws RuntimeException if the displacement is out of road bounds
     */
    public Triplet<ParametricCurve, Integer, Double> getSegment3Disp(double d){
        if(!MathUtils.dispInRange(d, 0, totalLength)){
            throw new DispOfBoundsException(d, 0, totalLength, this);
        }

        int index = binSearchRecursive(0, n, d);
        return new Triplet<>(segments.get(index), index, FastMath.abs(d - ((index == 0)?(0):(lenarr[index - 1]))));
    }

    /**
     * Searches the correct segment by road displacement.
     * @param d displacement in cm
     * @return a {@code Triplet<Spline, Double, Boolean>} containing the segment as a {@code Spline}, the relative displacement on that segment
     * and a boolean indicating if the road is continuous in the vicinity of the given point
     * @throws RuntimeException if the displacement is out of road bounds
     */
    public Triplet<ParametricCurve, Double, Boolean> getSegmentCont3Disp(double d){
        if(!MathUtils.dispInRange(d, 0, totalLength)){
            throw new DispOfBoundsException(d, 0, totalLength, this);
        }

        int index = binSearchRecursive(0, n, d);
        double delta_d = FastMath.abs(d - ((index == 0)?(0):(lenarr[index - 1])));
        return new Triplet<>(segments.get(index), delta_d, (delta_d < eps)?(isContinous.get(index)):(true));
    }

    /**
     * Searches the correct segment by road displacement.
     * @param d displacement in cm
     * @return the segment index
     * @throws RuntimeException if the displacement is out of road bounds
     */
    public int getSegment1Disp(double d){
        if(!MathUtils.dispInRange(d, 0, totalLength)){
            throw new DispOfBoundsException(d, 0, totalLength, this);
        }

        int index = binSearchRecursive(0, n, d);
        return index;
    }

    public Point2d pointAtDisp(double d){
        Pair<ParametricCurve, Double> output = getSegment2Disp(d);
        return output.getFirst().pointAtDisp(output.getSecond());
    }


    @Useless
    public double slopeAtDisp(double d){
        Pair<ParametricCurve, Double> output = getSegment2Disp(d);
        ParametricCurve s = output.getFirst();
        return s.tanAt(s.uAtDisp(output.getSecond()));
    }

    /**
     * This function returns the derivative on both x and y a given displacement on the road
     * If the requested point on the road is in the vicinity of a 2-segment binding that breaks continuity,
     * the function returns {@code null}.
     * @param d displacement in cm
     * @return a new {@code @Nullable Point2d} object
     */
    public @Nullable Point2d getDerivAtDisp(double d){
        Triplet<ParametricCurve, Double, Boolean> segment = this.getSegmentCont3Disp(d);
        ParametricCurve s = segment.getValue0();
        if(! segment.getValue2()){ // segment breaks continuity
            return null;
        }
        return s.derivAt(s.uAtDisp(segment.getValue1()));
    }

    /**
     * This function simply returns the derivative on x and y at a given displacement on the road,
     * unlike {@link #getDerivAtDisp(double)} which can also return {@code null}.
     * @param d displacement in cm
     * @return a new {@code @Nonnull Point2d} object
     */
    public @Nonnull Point2d getDerivAtDispNonnull(double d){
        Triplet<ParametricCurve, Double, Boolean> segment = this.getSegmentCont3Disp(d);
        ParametricCurve s = segment.getValue0();
        return s.derivAt(s.uAtDisp(segment.getValue1()));
    }

    /**
     * Returns the curvature of the road at a given displacement. The curvature is computed using the following
     * formula: {@code ((x', y') * (x'', y'')) / |(x', y')|^3} where * is the cross product of the vectors, x' and y'
     * are the values of the first derivatives and x'' and y'' are the values of the second derivatives.
     * @param d displacement in cm
     * @return the curvature as a {@code Double}
     */
    public @Nullable Double getCurvatureAtDisp(double d){
        Pair<ParametricCurve, Double> output = getSegment2Disp(d);
        ParametricCurve s = output.getFirst();

        double u = s.uAtDisp(output.getSecond());
        Vector2d vec1stDeriv = s.derivAt(u).toVector();
        Vector2d vec2ndDeriv = s.deriv2ndAt(u).toVector();

        return vec1stDeriv.cross(vec2ndDeriv) / FastMath.pow(vec1stDeriv.abs(), 3);
    }

    /**
     * Retrieves the total road length comprised of the sum of all segments' lengths.
     * @return the total length
     */
    public double getLength(){
        return totalLength;
    }

    /**
     * Retrieves the total number of segments.
     * @return the number of segments
     */
    public int getSegmentNum(){
        return n;
    }

    public ParametricCurve getSegmentByIndex(int idx){
        return segments.get(idx);
    }

    /**
     * Returns the heading retrieved from the {@code HeadingInterpolation} at a given displacement.
     * @param d displacement in cm
     * @return a {@code Vector2d}
     */
    public Vector2d getHeadingAtDisp(double d){
        Triplet<ParametricCurve, Integer, Double> ret = getSegment3Disp(d);

        ParametricCurve s = ret.getValue0();
        double param = s.uAtDisp(ret.getValue2());
        HeadingInterpolation interp = getHeadingInterpByIndex(ret.getValue1());

        return interp.getVectorAtParam(param);
    }

    public Vector2d getHeadingNormedAtDisp(double d){
        return getHeadingAtDisp(d).norm();
    }

//    public Point2d getDerivAtDisplacement(double d){
//        Pair<Spline, Double> output = getSegmentAtDisplacement(d);
//        Spline s = output.getFirst();
//
//        Point2d deriv = s.firstDerivativeAt(output.getSecond());
//        double abs = deriv.toVector().abs();
//
//        return new Point2d(deriv.getX() / abs, deriv.getY() / abs);
//    }
}
