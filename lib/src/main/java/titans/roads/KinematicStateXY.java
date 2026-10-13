package titans.roads;

import titans.geometry.Point2d;
import titans.geometry.Vector2d;

// unimplemented
public class KinematicStateXY {
    private final double time;
    private final Point2d position;
    private final Vector2d velocity;
    private final Vector2d acceleration;

    public KinematicStateXY(Point2d pos, Vector2d vel, Vector2d acc){
        position = pos;
        velocity = vel;
        acceleration = acc;
        time = 0;
    }

    public KinematicStateXY(double t, Point2d pos, Vector2d vel, Vector2d acc){
        position = pos;
        velocity = vel;
        acceleration = acc;
        time = t;
    }

    public double getTime(){
        return time;
    }

    public Point2d getPos(){
        return position;
    }

    public Vector2d getVel(){
        return velocity;
    }

    public Vector2d getAcc(){
        return acceleration;
    }

    public static KinematicStateXY getLinearInterpolation(KinematicStateXY first, KinematicStateXY second, double timetotal, double timefromfirst){
        Vector2d possum = first.position.toVector().add(second.position.toVector());
        Vector2d velsum = first.velocity.add(second.velocity);
        Vector2d accsum = first.acceleration.add(second.acceleration);

        double interpolation_factor = timefromfirst / timetotal;

        possum.mlt(interpolation_factor);
        velsum.mlt(interpolation_factor);
        accsum.mlt(interpolation_factor);

        return new KinematicStateXY(possum.toPoint(), velsum, accsum);
    }

    @Override
    public boolean equals(Object obj){
        if (obj instanceof KinematicStateXY kst){
            return (
                    kst.position.equals(this.position) &&
                    kst.velocity.equals(this.velocity) &&
                    kst.acceleration.equals(this.acceleration) &&
                    kst.time == this.time
            );
        } else {
            return false;
        }
    }
}
