package robot.subsystems.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Constants;

import java.util.List;

import static robot.Robot.drivetrain;

public class trajectoryTrackerCommand extends Command {

    private TrajectoryTracker trajectoryTrack;
    private List<Pose2d> waypoints;
    private double startingVelocity;
    private double endingVelocity;
    private TimedTrajectory<Pose2dWithCurvature> trajectory;
    private boolean finished = false;
    private boolean reversed = false;
    private double maxVelocity = Constants.Drivetrain.MAX_VELOCITY;
    private double maxAcceleration = Constants.Drivetrain.MAX_ACCEL;
    private boolean newRam;

    public trajectoryTrackerCommand(List<Pose2d> waypoints, double startingVelocity, double endingVelocity, boolean reversed, boolean newRam) {
        trajectoryTrack = drivetrain.getTrajectoryTracker();
        this.waypoints = waypoints;
        this.startingVelocity = startingVelocity;
        this.endingVelocity = endingVelocity;
        this.reversed = reversed;
        this.newRam = newRam;

    }

    @Override
    protected void initialize() {
        if (waypoints != null) {
            waypoints.add(0, drivetrain.localization.getRobotPosition());
            this.trajectory = drivetrain.generateTrajectory(waypoints, startingVelocity, endingVelocity, reversed);

        }
        drivetrain.trajectoryTracker.reset(trajectory);

        LiveDashboard.INSTANCE.setFollowingPath(true);
    }

    @Override
    protected void execute() {
        SmartDashboard.putNumber("x distance", drivetrain.localization.getRobotPosition().getTranslation().getX().getValue());
        TrajectoryTrackerOutput trackerOutput = drivetrain.trajectoryTracker.nextState(drivetrain.localization.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));
        if (newRam) {
            drivetrain.setOutput(trackerOutput);
        } else {
            double linearVelocity = trackerOutput.getLinearVelocity().getValue();//m/s
            double angularVelocity = trackerOutput.getAngularVelocity().getValue();//m/s


            double tangenialVelocity = Constants.Drivetrain.ROBOT_WIDTH / 2.0 * angularVelocity; // Multiply angular velocity by the robot radius to get the tangenial velocity

            double leftVelocity = linearVelocity - tangenialVelocity;
            double rightVelocity = linearVelocity + tangenialVelocity;

            drivetrain.setVelocity(leftVelocity, rightVelocity);
        }
        TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTrack.getReferencePoint();

        if (referencePoint != null) {
            Pose2d referencePose = referencePoint.getState().getState().getPose();

            LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
            LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
            LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());
        }
    }

    @Override
    protected boolean isFinished() {
        return trajectoryTrack.isFinished();

    }

    @Override
    protected void end() {
        drivetrain.zeroOutputs();
        LiveDashboard.INSTANCE.setFollowingPath(false);
        System.out.println("done");
    }

    @Override
    protected void interrupted() {

    }
}
