package robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.Utilities;
import robot.utilities.TalonConfiguration;

public class Drivetrain extends Subsystem {

    private final TalonSRX testMotor = new TalonSRX(11);
    public Drivetrain() {
        TalonConfiguration configs = new TalonConfiguration();
        configs.motorConfigs.motionCurveStrength = 5;
        configs.motorConfigs.motionCruiseVelocity = 2;
        configs.setNeutralMode(NeutralMode.Brake);
        Utilities.configAllTalons(configs, testMotor);
        System.out.println(configs.getNeutralMode());
        testMotor.setNeutralMode(NeutralMode.Brake);
    }




    @Override
    protected void initDefaultCommand() {

    }
}