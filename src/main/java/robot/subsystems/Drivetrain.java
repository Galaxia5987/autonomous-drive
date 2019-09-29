package robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team254.lib.physics.DCMotorTransmission;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
import robot.utilities.Point;

import static robot.Robot.navx;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
// Put methods for controlling this subsystem
// here. Call these from Commands.

    public Point currentLocation = new Point(0, 0);
    NativeUnitLengthModel nativeUnitModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(drivetrainConstants.TICKS_PER_ROTATION), LengthKt.getMeter(drivetrainConstants.WHEEL_RADIUS));
    private final FalconSRX<Length> leftMaster = new FalconSRX<Length>(0, nativeUnitModel, TimeUnitsKt.getMillisecond(10));
    private final FalconSRX<Length> rightMaster = new FalconSRX<Length>(0, nativeUnitModel, TimeUnitsKt.getMillisecond(10));

    private final VictorSPX leftSlave1 = new VictorSPX(0);
    private final VictorSPX leftSlave2 = new VictorSPX(0);

    private final VictorSPX rightSlave1 = new VictorSPX(0);
    private final VictorSPX rightSlave2 = new VictorSPX(0);

    public Localization localization = new TankEncoderLocalization(
            () -> Rotation2dKt.getDegree(getAngle()),
            () -> LengthKt.getMeter(getLeftDistance()),
            () -> LengthKt.getMeter(getRightDistance())
    );

    public TrajectoryTracker trajectoryTracker = new RamseteTracker(drivetrainConstants.kBeta, drivetrainConstants.kZeta);


    public Drivetrain() {
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);


        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);

        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);


    }

    public static final DCMotorTransmission leftTransmissionModel = new DCMotorTransmission(
            1 / drivetrainConstants.kADriveLeftLow,
            Math.pow(drivetrainConstants.WHEEL_RADIUS, drivetrainConstants.WHEEL_RADIUS) * drivetrainConstants.ROBOT_MASS / (2.0 * drivetrainConstants.kADriveLeftLow),
            drivetrainConstants.kVDriveLeftLow);

    public static final DCMotorTransmission rightTransmissionModel = new DCMotorTransmission(
            1 / drivetrainConstants.kADriveRightLow,
            Math.pow(drivetrainConstants.WHEEL_RADIUS, drivetrainConstants.WHEEL_RADIUS) * drivetrainConstants.ROBOT_MASS / (2.0 * drivetrainConstants.kADriveRightLow),
            drivetrainConstants.kVDriveRightLow);

    public double getRightDistance() {
        return rightMaster.getSelectedSensorPosition() / drivetrainConstants.TICKS_PER_METER;
    }

    public double getLeftDistance() {
        return leftMaster.getSelectedSensorPosition() / drivetrainConstants.TICKS_PER_METER;
    }

    public double getAngle() {
        return navx.getAngle();
    }


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}