package robot.subsystems.Commands;

import edu.wpi.first.wpilibj.command.Command;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import robot.subsystems.drivetrainConstants;

import java.util.ArrayList;

import static robot.Robot.drivetrain;

public class trajectoryTrackerCommand extends Command {

    private TrajectoryTracker trajectoryTrack;
    private ArrayList<Pose2d> waypoints;
    private double startingVelocity;
    private double endingVelocity;
    private TimedTrajectory<Pose2dWithCurvature> trajectory;
    private boolean finished = false;
    private boolean reversed = false;
    private double maxVelocity = drivetrainConstants.MAX_VELOCITY;
    private double maxAcceleration = drivetrainConstants.MAX_ACCEL;

    public trajectoryTrackerCommand(ArrayList<Pose2d> waypoints, double startingVelocity, double endingVelocity, boolean reversed){
        trajectoryTrack = drivetrain.getTrajectoryTracker();
        this.waypoints = waypoints;
        this.startingVelocity = startingVelocity;
        this.endingVelocity = endingVelocity;
        this.reversed = reversed;

    }
    @Override
    protected void initialize() {
        if (waypoints != null) {
            waypoints.add(0, drivetrain.localization.getRobotPosition());
            this.trajectory = drivetrain.generateTrajectory(waypoints, startingVelocity, endingVelocity, reversed);
        }
        LiveDashboard.INSTANCE.setFollowingPath(true);

    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
