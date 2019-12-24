package robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.utilities.CustomTalonConfigs;

import static robot.Robot.navx;

public class Drivetrain extends Subsystem {

    private final TalonSRX testMotor = new TalonSRX(0);
    public Drivetrain() {
        CustomTalonConfigs customConfigs = new CustomTalonConfigs();
        customConfigs.motorConfigs.motionCurveStrength = 6;
        customConfigs.motorConfigs.motionCruiseVelocity = 2;

    }

    public double getRightVelocity() {
        return rightMaster.getSensorVelocity().getValue();
    }

    public void allMotorsToCoast(){
        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave1.setNeutralMode(NeutralMode.Coast);
        leftSlave2.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave1.setNeutralMode(NeutralMode.Coast);
        rightSlave2.setNeutralMode(NeutralMode.Coast);
    }

    public void enableVoltageCompensation(){
        rightMaster.enableVoltageCompensation(true);
        leftMaster.enableVoltageCompensation(true);
        leftSlave1.enableVoltageCompensation(true);
        leftSlave2.enableVoltageCompensation(true);
        rightSlave1.enableVoltageCompensation(true);
        rightSlave2.enableVoltageCompensation(true);
    }

    public void setVoltageCompensationSaturation(double value){
        rightMaster.configVoltageCompSaturation(value);
        leftMaster.configVoltageCompSaturation(value);
        leftSlave1.configVoltageCompSaturation(value);
        leftSlave2.configVoltageCompSaturation(value);
        rightSlave1.configVoltageCompSaturation(value);
        rightSlave2.configVoltageCompSaturation(value);
    }

    public double getLeftVelocity() {
        return leftMaster.getSensorVelocity().getValue();

    }

    public double getAngle() {
        return -navx.getAngle();
    }


    @NotNull
    public FalconMotor<Length> getLeftMotor() {
        return leftMaster;
    }

    @NotNull
    public FalconMotor<Length> getRightMotor() {
        return rightMaster;
    }


    public void setVelocity(double leftVelocity, double rightVelocity) {
        leftMaster.setVelocity(VelocityKt.getVelocity(LengthKt.getMeter(leftVelocity)));
        rightMaster.setVelocity(VelocityKt.getVelocity(LengthKt.getMeter(rightVelocity)));
    }

    public void setSpeed(double leftPower, double rightPower) {
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }


    @Override
    protected void initDefaultCommand() {

    }
}