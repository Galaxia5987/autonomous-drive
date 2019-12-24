package robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import robot.utilities.TalonConfiguration;
import robot.utilities.CustomVictorConfigs;

public class Utilities {
    public static void configAllTalons(TalonConfiguration configs, TalonSRX... talons) {
        for (TalonSRX talon : talons) {
            talon.configAllSettings(configs.motorConfigs);
            talon.setNeutralMode(configs.getNeutralMode());
            talon.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            talon.enableVoltageCompensation(configs.isEnableVoltageCompensation());

        }

    }

    public static void configAllVictors(CustomVictorConfigs configs, VictorSPX... victors) {
        for (VictorSPX victor : victors) {
            victor.configAllSettings(configs.motorConfigs);
            victor.setNeutralMode(configs.getNeutralMode());
            victor.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            victor.enableVoltageCompensation(configs.isEnableVoltageCompensation());

        }
    }
}
