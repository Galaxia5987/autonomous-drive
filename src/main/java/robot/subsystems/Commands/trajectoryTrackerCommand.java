package robot.subsystems.Commands;

import edu.wpi.first.wpilibj.command.Command;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;

import static robot.Robot.drivetrain;

public class trajectoryTrackerCommand extends Command {

    private TrajectoryTracker trajectoryTrack;


    public trajectoryTrackerCommand(){
        trajectoryTrack = drivetrain.getTrajectoryTracker();
    }
    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        trajectoryTrack.state
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
