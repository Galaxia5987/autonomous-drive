package robot;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;

import java.util.ArrayList;
import java.util.List;

import static robot.Utilities.metersToRadians;

/**
 * A class holding all of the constants of every mechanism on the robot.
 * Place global constants in this class, and mechanism-specific constants inside their respective mechanism subclass.
 * When accessing a mechanism-specific port, call Constants.[MECHANISM].[CONSTANT]
 */
public class Constants {


    public static class Drivetrain {

    }
}
