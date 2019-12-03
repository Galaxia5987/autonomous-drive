package robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.utilities.CustomTalonConfigs;
import robot.utilities.CustomVictorConfigs;

public class Utilities {
    /**
     * convert meters to radians of the drivetrain wheels
     *
     * @param meters distance in meters.
     * @return wheel radians.
     */
    public static double metersToRadians(double meters) {
        return meters * Constants.Drivetrain.WHEEL_DIAMATER / 2;
    }

    /**
     * A cleaner way to create a Pose2d object, since we work in meters and degrees.
     * the name is temporary
     * @param x x value of the Pose2d object in meters
     * @param y y value of the Pose2d object in meters
     * @param angle rotation of the point in degrees
     * @return GHRobotics Pose2d object.
     */
    public static Pose2d genPoint(double x, double y, double angle){
        return new Pose2d(LengthKt.getMeter(x), LengthKt.getMeter(y), Rotation2dKt.getDegree(angle));
    }


    public static void configAllTalons(CustomTalonConfigs configs, TalonSRX... talons){
        for (TalonSRX talon: talons) {
            talon.configAllSettings(configs.motorConfigs);
            talon.setNeutralMode(configs.getNeutralMode());
            talon.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            talon.enableVoltageCompensation(configs.isEnableVoltageCompensation());

        }

    }
    public static void configAllVictors(CustomVictorConfigs configs, VictorSPX ... victors){
        for (VictorSPX victor: victors){
            victor.configAllSettings(configs.motorConfigs);
            victor.setNeutralMode(configs.getNeutralMode());
            victor.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            victor.enableVoltageCompensation(configs.isEnableVoltageCompensation());

        }
    }
}
