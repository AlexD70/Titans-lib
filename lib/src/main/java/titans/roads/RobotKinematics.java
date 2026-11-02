package titans.roads;

import org.javatuples.Triplet;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.Optional;

public interface RobotKinematics {
    double getVelocityLimit(double displacement, Road road, @Nullable Vector2d prevTan, int lastSegment);
    double getLinearAcceleration(double currentVel, @Nullable Double curvature);
    void setWheelOffsets(double xWheelOffset, double yWheelOffset);
    double maxVel();
    double maxAcc();
    double maxAngVel();
    double maxAngAcc();
}
