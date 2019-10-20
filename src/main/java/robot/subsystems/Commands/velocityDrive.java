package robot.subsystems.Commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static robot.Robot.drivetrain;

/**
 *
 */
public class velocityDrive extends Command {

    double leftVel;
    double rightVel;
    public velocityDrive(double leftVel, double rightVel) {
        this.leftVel = leftVel;
        this.rightVel = rightVel;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        drivetrain.setVelocity(leftVel, rightVel);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        System.out.println(drivetrain.getRightVelocity() + " " + drivetrain.getLeftVelocity());

        drivetrain.setVelocity(leftVel, rightVel);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setVelocity(0,0);
    }

    // Called when another command which requires one or more of the same
// subsystems is scheduled to run
    protected void interrupted() {
    }
}