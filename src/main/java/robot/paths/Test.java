package robot.paths;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import java.util.Arrays;
import java.util.List;

import static robot.Utilities.genPoint;

public class Test {

    public static List<Pose2d> points = Arrays.asList(
            genPoint(0, 0, 0),
            genPoint(1, 0, 0)
    );

}
