package robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import edu.wpi.first.wpilibj.Notifier;
import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem;
import org.ghrobotics.lib.wrappers.FalconMotor;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
import org.jetbrains.annotations.NotNull;
import robot.Constants;
import robot.utilities.CustomTalonConfigs;

import java.util.List;

import static robot.Robot.navx;

public class Drivetrain extends TankDriveSubsystem {


    private NativeUnitLengthModel nativeUnitModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(Constants.Drivetrain.TICKS_PER_ROTATION), LengthKt.getMeter(Constants.Drivetrain.WHEEL_DIAMATER));
    private final FalconSRX<Length> leftMaster = new FalconSRX<>(3, nativeUnitModel, TimeUnitsKt.getMillisecond(10));
    private final FalconSRX<Length> rightMaster = new FalconSRX<>(6, nativeUnitModel, TimeUnitsKt.getMillisecond(10));
    private final VictorSPX leftSlave1 = new VictorSPX(4);
    private final VictorSPX leftSlave2 = new VictorSPX(5);
    private final VictorSPX rightSlave1 = new VictorSPX(7);
    private final VictorSPX rightSlave2 = new VictorSPX(8);





    public Localization localization = new TankEncoderLocalization(
            () -> Rotation2dKt.getDegree(getAngle()),
            leftMaster::getSensorPosition,
            rightMaster::getSensorPosition
    );


    public TrajectoryTracker trajectoryTracker = new RamseteTracker(Constants.Drivetrain.kBeta, Constants.Drivetrain.kZeta);


    public Drivetrain(boolean newRam) {


        Notifier localizationNotifier = new Notifier(() -> {
            localization.update();
        });
        localizationNotifier.startPeriodic(1d / 100d);

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);


        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);

        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);


        if (!newRam) {
            leftMaster.config_kP(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[0]);
            leftMaster.config_kI(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[1]);
            leftMaster.config_kD(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[2]);
            leftMaster.config_kF(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID[3]);

            rightMaster.config_kP(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[0]);
            rightMaster.config_kI(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[1]);
            rightMaster.config_kD(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[2]);
            rightMaster.config_kF(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID[3]);
        } else {
            leftMaster.config_kP(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID_MODEL[0]);
            leftMaster.config_kI(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID_MODEL[1]);
            leftMaster.config_kD(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID_MODEL[2]);
            leftMaster.config_kF(0, Constants.Drivetrain.LEFT_TALON_VELOCITY_PID_MODEL[3]);

            rightMaster.config_kP(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID_MODEL[0]);
            rightMaster.config_kI(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID_MODEL[1]);
            rightMaster.config_kD(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID_MODEL[2]);
            rightMaster.config_kF(0, Constants.Drivetrain.RIGHT_TALON_VELOCITY_PID_MODEL[3]);
        }



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

    public static final DCMotorTransmission leftTransmissionModel = new DCMotorTransmission(1 / Constants.Drivetrain.kVDriveLeftLow,
            Constants.Drivetrain.WHEEL_DIAMATER * Constants.Drivetrain.WHEEL_DIAMATER * Constants.Drivetrain.ROBOT_MASS / (2.0 * Constants.Drivetrain.kADriveLeftLow),
            Constants.Drivetrain.kVInterceptLeftLow);


    public static final DCMotorTransmission rightTransmissionModel = new DCMotorTransmission(1 / Constants.Drivetrain.kVDriveRightLow,
            Constants.Drivetrain.WHEEL_DIAMATER * Constants.Drivetrain.WHEEL_DIAMATER * Constants.Drivetrain.ROBOT_MASS / (2.0 * Constants.Drivetrain.kADriveRightLow),
            Constants.Drivetrain.kVInterceptRightLow);


    public static final DifferentialDrive driveModel = new DifferentialDrive(
            Constants.Drivetrain.ROBOT_MASS,
            Constants.Drivetrain.MOMENT_OF_INERTIA,
            Constants.Drivetrain.ANGULAR_DRAG,
            Constants.Drivetrain.WHEEL_DIAMATER,
            Constants.Drivetrain.ROBOT_WIDTH / 2,
            leftTransmissionModel,
            rightTransmissionModel
    );

    public double getAngle() {
        return -navx.getAngle();
    }

    public TrajectoryTracker getTrajectoryTracker() {
        return trajectoryTracker;
    }


    private static final DifferentialDrive DIFFERENTIAL_DRIVE = new DifferentialDrive(
            Constants.Drivetrain.ROBOT_MASS,
            Constants.Drivetrain.MOMENT_OF_INERTIA,
            Constants.Drivetrain.ANGULAR_DRAG,
            Constants.Drivetrain.WHEEL_DIAMATER,
            Constants.Drivetrain.ROBOT_WIDTH / 2.0,
            leftTransmissionModel,
            rightTransmissionModel
    );


    public TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, double startingVelocity, double endingVelocity, boolean reversed) {
        return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(waypoints, Constants.Drivetrain.constraints,
                VelocityKt.getVelocity(LengthKt.getMeter(startingVelocity)),
                VelocityKt.getVelocity(LengthKt.getMeter(endingVelocity)),
                VelocityKt.getVelocity(LengthKt.getMeter(Constants.Drivetrain.MAX_VELOCITY)),
                AccelerationKt.getAcceleration(LengthKt.getMeter(Constants.Drivetrain.MAX_ACCEL)),
                reversed,
                true
        );
    }

    @NotNull
    @Override
    public Localization getLocalization() {
        return localization;
    }

    @NotNull
    @Override
    public DifferentialDrive getDifferentialDrive() {
        return DIFFERENTIAL_DRIVE;
    }

    @NotNull
    @Override
    public FalconMotor<Length> getLeftMotor() {
        return leftMaster;
    }

    @NotNull
    @Override
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


}