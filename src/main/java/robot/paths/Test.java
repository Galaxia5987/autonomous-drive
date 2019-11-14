package robot.paths;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import java.util.Arrays;
import java.util.List;

public class Test {

    public static List<Pose2d> points = Arrays.asList(
            new Pose2d(LengthKt.getMeter(0), LengthKt.getMeter(0), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getMeter(1), LengthKt.getMeter(0), Rotation2dKt.getDegree(0))
    );

}
