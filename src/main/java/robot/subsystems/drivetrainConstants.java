package robot.subsystems;

import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import robot.Constants;

public class drivetrainConstants {

    public static double mToR(double in) {
        return in * Constants.Drivetrain.WHEEL_DIAMATER * Math.PI / 2;
    }
//model




}
