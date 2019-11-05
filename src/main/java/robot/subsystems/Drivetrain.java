package robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

import java.util.List;

import static robot.Robot.navx;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends TankDriveSubsystem {

// Put methods for controlling this subsystem
// here. Call these from Commands.

    private NativeUnitLengthModel nativeUnitModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(drivetrainConstants.TICKS_PER_ROTATION), LengthKt.getMeter(drivetrainConstants.WHEEL_RADIUS));
    public final FalconSRX<Length> leftMaster = new FalconSRX<>(3, nativeUnitModel, TimeUnitsKt.getMillisecond(10));
    public final FalconSRX<Length> rightMaster = new FalconSRX<>(6, nativeUnitModel, TimeUnitsKt.getMillisecond(10));

    public Localization localization = new TankEncoderLocalization(
            () -> Rotation2dKt.getDegree(getAngle()),
            leftMaster::getSensorPosition,
            rightMaster::getSensorPosition
    );


    public TrajectoryTracker trajectoryTracker = new RamseteTracker(drivetrainConstants.kBeta, drivetrainConstants.kZeta);


    public Drivetrain(boolean newRam) {



        Notifier localizationNotifier = new Notifier(() -> {
            localization.update();
        });
        localizationNotifier.startPeriodic(1d / 100d);

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);


        VictorSPX leftSlave1 = new VictorSPX(4);
        leftSlave1.follow(leftMaster);
        VictorSPX leftSlave2 = new VictorSPX(5);
        leftSlave2.follow(leftMaster);

        VictorSPX rightSlave1 = new VictorSPX(7);
        rightSlave1.follow(rightMaster);
        VictorSPX rightSlave2 = new VictorSPX(8);
        rightSlave2.follow(rightMaster);


        if (!newRam) {
            leftMaster.config_kP(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID[0]);
            leftMaster.config_kI(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID[1]);
            leftMaster.config_kD(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID[2]);
            leftMaster.config_kF(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID[3]);

            rightMaster.config_kP(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID[0]);
            rightMaster.config_kI(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID[1]);
            rightMaster.config_kD(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID[2]);
            rightMaster.config_kF(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID[3]);
        }else {
            leftMaster.config_kP(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID_MODEL[0]);
            leftMaster.config_kI(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID_MODEL[1]);
            leftMaster.config_kD(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID_MODEL[2]);
            leftMaster.config_kF(0, drivetrainConstants.LEFT_TALON_VELOCITY_PID_MODEL[3]);

            rightMaster.config_kP(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID_MODEL[0]);
            rightMaster.config_kI(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID_MODEL[1]);
            rightMaster.config_kD(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID_MODEL[2]);
            rightMaster.config_kF(0, drivetrainConstants.RIGHT_TALON_VELOCITY_PID_MODEL[3]);
        }

        rightMaster.configVoltageCompSaturation(11.0);
        leftMaster.configVoltageCompSaturation(11.0);
        leftSlave1.configVoltageCompSaturation(11.0);
        leftSlave2.configVoltageCompSaturation(11.0);
        rightSlave1.configVoltageCompSaturation(11.0);
        rightSlave2.configVoltageCompSaturation(11.0);

        rightMaster.enableVoltageCompensation(true);
        leftMaster.enableVoltageCompensation(true);
        leftSlave1.enableVoltageCompensation(true);
        leftSlave2.enableVoltageCompensation(true);
        rightSlave1.enableVoltageCompensation(true);
        rightSlave2.enableVoltageCompensation(true);
//        leftMaster.configSelectedFeedbackCoefficient(1.0/1935.0);
//        rightMaster.configSelectedFeedbackCoefficient(1.0/1935.0);


        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftSlave1.setNeutralMode(NeutralMode.Brake);
        leftSlave2.setNeutralMode(NeutralMode.Brake);
        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightSlave1.setNeutralMode(NeutralMode.Brake);
        rightSlave2.setNeutralMode(NeutralMode.Brake);

        rightMaster.setInverted(true);
        rightSlave1.setInverted(true);
        rightSlave2.setInverted(true);
    }

    public double getRightVelocity(){
        return rightMaster.getSensorVelocity().getValue();
    }

    public double getLeftVelocity(){
        return leftMaster.getSensorVelocity().getValue();

    }

    private static final DCMotorTransmission leftTransmissionModel = new DCMotorTransmission(1 / drivetrainConstants.kVDriveLeftLow,
            drivetrainConstants.WHEEL_RADIUS * drivetrainConstants.WHEEL_RADIUS * drivetrainConstants.ROBOT_MASS / (2.0 * drivetrainConstants.kADriveLeftLow),
            drivetrainConstants.kVInterceptLeftLow);


    private static final DCMotorTransmission rightTransmissionModel = new DCMotorTransmission(1 / drivetrainConstants.kVDriveRightLow,
            drivetrainConstants.WHEEL_RADIUS * drivetrainConstants.WHEEL_RADIUS * drivetrainConstants.ROBOT_MASS / (2.0 * drivetrainConstants.kADriveRightLow),
            drivetrainConstants.kVInterceptRightLow);


    public double getAngle() {
        return -navx.getAngle();
    }

    public TrajectoryTracker getTrajectoryTracker() {
        return trajectoryTracker;
    }


    private static final DifferentialDrive DIFFERENTIAL_DRIVE = new DifferentialDrive(
            drivetrainConstants.ROBOT_MASS,
            drivetrainConstants.MOMENT_OF_INERTIA,
            drivetrainConstants.ANGULAR_DRAG,
            drivetrainConstants.WHEEL_RADIUS,
            drivetrainConstants.ROBOT_WIDTH / 2.0,
            leftTransmissionModel,
            rightTransmissionModel
    );


    public TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, double startingVelocity, double endingVelocity, boolean reversed) {
        return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(waypoints, drivetrainConstants.constraints,
                VelocityKt.getVelocity(LengthKt.getMeter(startingVelocity)),
                VelocityKt.getVelocity(LengthKt.getMeter(endingVelocity)),
                VelocityKt.getVelocity(LengthKt.getMeter(drivetrainConstants.MAX_VELOCITY)),
                AccelerationKt.getAcceleration(LengthKt.getMeter(drivetrainConstants.MAX_ACCEL)),
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


    public void setVelocity(double leftVelocity, double rightVelocity){
        leftMaster.setVelocity(VelocityKt.getVelocity(LengthKt.getMeter(leftVelocity)));
        rightMaster.setVelocity(VelocityKt.getVelocity(LengthKt.getMeter(rightVelocity)));
    }
    public void setSpeed(double leftPower, double rightPower){
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }


}