package robot; /**
 * This is a very simple robot program that can be used to send telemetry to
 * the data_logger script to characterize your drivetrain. If you wish to use
 * your actual robot code, you only need to implement the simple logic in the
 * autonomousPeriodic function and change the NetworkTables update rate
 * <p>
 * This program assumes that you are using TalonSRX motor controllers and that
 * the drivetrain encoders are attached to the TalonSRX
 */


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
    public static OI m_oi;
    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    @Override
    public void robotInit() {

    }



    @Override
    public void disabledInit(){
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {

    }


    @Override
    public void autonomousPeriodic() {


    }
}