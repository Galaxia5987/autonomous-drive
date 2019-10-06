package robot.subsystems;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import java.util.ArrayList;
import java.util.List;

public class drivetrainConstants {
    public static final double TICKS_PER_METER = 0;
    public static final double TICKS_PER_ROTATION = 0;
    public static final double ROBOT_MASS = 0; //Robot Mass + 5kg for the battery + 2kg for the bumpers

    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCEL = 0;

    public static final double WHEEL_RADIUS = 0; // meters. TODO tune
    public static final double ROBOT_WIDTH = 0; // meters

    public static final double kBeta = 0;
    public static final double kZeta = 0;

    public static final double kVDriveLeftLow = 0; // Volts per radians per second - Calculated emperically
    public static final double kADriveLeftLow = 0; // Volts per radians per second per second
    public static final double kVInterceptLeftLow = 0; // Volts

    public static final double kVDriveRightLow = 0; // Volts per radians per second - Calculated emperically
    public static final double kADriveRightLow = 0; // Volts per radians per second per second
    public static final double kVInterceptRightLow = 0; // Volts

    public static final double MOMENT_OF_INERTIA = 10;// kg m^2
    public static final double ANGULAR_DRAG = 12;// N*m / (rad/sec)


    //******************
    //***Constraints****
    //******************
    public static final List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();

    private static final double ACCELERATION_CONSTRAINT = 1.2;
    private static final double VELOCITY_CONSTRAINT = 3;
    private static final double RECTANGLE_1 = 4;
    private static final double RECTANGLE_2 = 7;
    private static final double RECTANGLE_3 = 8;
    private static final double RECTANGLE_4 = 20;

    static {
        constraints.add(new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getMeter(ACCELERATION_CONSTRAINT))));
        constraints.add(new VelocityLimitRegionConstraint(new Rectangle2d(LengthKt.getFeet(RECTANGLE_1), LengthKt.getFeet(RECTANGLE_2), LengthKt.getFeet(RECTANGLE_3), LengthKt.getFeet(RECTANGLE_4)), VelocityKt.getVelocity(LengthKt.getFeet(VELOCITY_CONSTRAINT))));
    }

}
