package robot.subsystems.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Constants;

import static robot.Robot.drivetrain;

/**
 *
 */
public class velocityDrive extends Command {

    double leftVel;
    double rightVel;
    Timer timer = new Timer();

    public velocityDrive(double leftVel, double rightVel) {
        this.leftVel = leftVel;
        this.rightVel = rightVel;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (leftVel < 0)
            drivetrain.leftMaster.config_kF(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[3] * -1.5);
        else
            drivetrain.leftMaster.config_kF(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[3]);
        System.out.println("begin");
        timer.start();
        final double v = 0;
        drivetrain.setSpeed(v, v);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (timer.get() > 0) {

            drivetrain.setVelocity(leftVel, rightVel);
        }
        SmartDashboard.putNumber("right Vel", drivetrain.getRightVelocity());
        SmartDashboard.putNumber("left Vel", drivetrain.getLeftVelocity());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.get() > 20;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
        timer.reset();
    }

    // Called when another command which requires one or more of the same
// subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}