package robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.FalconMotor;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
import org.jetbrains.annotations.NotNull;
import robot.Constants;

import static robot.Robot.navx;

public class Drivetrain extends Subsystem {


    private NativeUnitLengthModel nativeUnitModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(Constants.Drivetrain.TICKS_PER_ROTATION), LengthKt.getMeter(Constants.Drivetrain.WHEEL_DIAMATER));
    private final FalconSRX<Length> leftMaster = new FalconSRX<>(3, nativeUnitModel, TimeUnitsKt.getMillisecond(10));
    private final FalconSRX<Length> rightMaster = new FalconSRX<>(6, nativeUnitModel, TimeUnitsKt.getMillisecond(10));
    private final VictorSPX leftSlave1 = new VictorSPX(4);
    private final VictorSPX leftSlave2 = new VictorSPX(5);
    private final VictorSPX rightSlave1 = new VictorSPX(7);
    private final VictorSPX rightSlave2 = new VictorSPX(8);


    public Drivetrain() {


        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);


        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);

        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);


        leftMaster.config_kP(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[0]);
        leftMaster.config_kI(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[1]);
        leftMaster.config_kD(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[2]);
        leftMaster.config_kF(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[3]);

        rightMaster.config_kP(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[0]);
        rightMaster.config_kI(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[1]);
        rightMaster.config_kD(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[2]);
        rightMaster.config_kF(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[3]);


        allMotorsToCoast();
        enableVoltageCompensation();
        setVoltageCompensationSaturation(11);

        rightMaster.setInverted(true);
        rightSlave1.setInverted(true);
        rightSlave2.setInverted(true);
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