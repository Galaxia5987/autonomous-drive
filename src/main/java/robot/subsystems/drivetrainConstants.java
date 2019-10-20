package robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
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
    public static final double TICKS_PER_ROTATION = 960;
    public static final double ROBOT_MASS = 59.7; //Robot Mass + 5kg for the battery + 2kg for the bumpers

    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCEL = 1;

    public static final double WHEEL_RADIUS = 0.1524; // meters. TODO tune
    public static final double ROBOT_WIDTH = 0.75; // meters

    public static final double kBeta = 2;
    public static final double kZeta = 0.7;

    public static final double kVDriveLeftLow = 1.94; // Volts per radians per second - Calculated emperically
    public static final double kADriveLeftLow = 0.836; // Volts per radians per second per second
    public static final double kVInterceptLeftLow = 1.42; // Volts

    public static final double kVDriveRightLow = 1.94; // Volts per radians per second - Calculated emperically
    public static final double kADriveRightLow = 0.836; // Volts per radians per second per second
    public static final double kVInterceptRightLow = 1.42; // Volts

    public static final double MOMENT_OF_INERTIA = 10;// kg m^2
    public static final double ANGULAR_DRAG = 12;// N*m / (rad/sec)

    public static final double[] TALON_VELOCITY_PID ={2.68, 0, 0, 0};//kP, kI, kD, kF


    //******************
    //***Constraints****
    //******************
    public static final List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();

    private static final double ACCELERATION_CONSTRAINT = 1.2;
    private static final double VELOCITY_CONSTRAINT = 3;


    static {
        constraints.add(new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getMeter(ACCELERATION_CONSTRAINT))));
    }

}
