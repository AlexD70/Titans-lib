package titans.geometry;

import org.apache.commons.math3.util.FastMath;

public class Pose {
    private Point2d point;
    private double theta;

    public Pose(double x, double y, double theta) {
        this.point = new Point2d(x, y);
        this.theta = theta;
    }

    public Pose(Point2d p, double theta){
        this.point = p;
        this.theta = theta;
    }

    public double getX(){
        return point.getX();
    }

    public double getY(){
        return point.getY();
    }

    public double getTheta(){
        return this.theta;
    }

    public Point2d getPoint(){
        return point;
    }

    public Vector2d getVector(){
        return point.toVector();
    }

    private static final double EPS = 0.001;
    @Override
    public boolean equals(Object obj){
        if(obj instanceof Pose pose){
            return (pose.point.equals(this.point) && (FastMath.abs(pose.theta - this.theta) < EPS));
        } else {
            return false;
        }
    }
}
