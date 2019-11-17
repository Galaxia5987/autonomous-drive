package robot;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

public class Utilities {
    /**
     * convert meters to radians of the drivetrain wheels
     *
     * @param meters distance in meters.
     * @return wheel radians.
     */
    public static double metersToRadians(double meters) {
        return meters * Constants.Drivetrain.WHEEL_DIAMATER / 2;
    }

    /**
     * A cleaner way to create a Pose2d object, since we work in meters and degrees.
     * the name is temporary
     * @param x x value of the Pose2d object in meters
     * @param y y value of the Pose2d object in meters
     * @param angle rotation of the point in degrees
     * @return GHRobotics Pose2d object.
     */
    public static Pose2d genPoint(double x, double y, double angle){
        return new Pose2d(LengthKt.getMeter(x), LengthKt.getMeter(y), Rotation2dKt.getDegree(angle));
    }
}
