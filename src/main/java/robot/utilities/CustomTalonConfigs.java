package robot.utilities;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

public class CustomTalonConfigs {


    private NeutralMode neutralMode;
    private FeedbackDevice feedbackDevice;
    private boolean enableVoltageCompensation;
    public TalonSRXConfiguration motorConfigs = new TalonSRXConfiguration();


    public CustomTalonConfigs() {

        neutralMode = NeutralMode.Coast;
        feedbackDevice = FeedbackDevice.CTRE_MagEncoder_Absolute;
        enableVoltageCompensation = false;


    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    public FeedbackDevice getFeedbackDevice() {
        return feedbackDevice;
    }

    public boolean isEnableVoltageCompensation() {
        return enableVoltageCompensation;
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }

    public void setFeedbackDevice(FeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
    }

    public void setEnableVoltageCompensation(boolean enableVoltageCompensation) {
        this.enableVoltageCompensation = enableVoltageCompensation;
    }

    public void setPrimaryPID(TalonSRXPIDSetConfiguration primaryPID) {
        this.motorConfigs.primaryPID = primaryPID;
    }

    public void setAuxiliaryPID(TalonSRXPIDSetConfiguration auxiliaryPID) {
        this.motorConfigs.auxiliaryPID = auxiliaryPID;
    }

    public void setForwardLimitSwitchSource(LimitSwitchSource forwardLimitSwitchSource) {
        this.motorConfigs.forwardLimitSwitchSource = forwardLimitSwitchSource;
    }

    public void setReverseLimitSwitchSource(LimitSwitchSource reverseLimitSwitchSource) {
        this.motorConfigs.reverseLimitSwitchSource = reverseLimitSwitchSource;
    }

    public void setForwardLimitSwitchDeviceID(int forwardLimitSwitchDeviceID) {
        this.motorConfigs.forwardLimitSwitchDeviceID = forwardLimitSwitchDeviceID;
    }

    public void setReverseLimitSwitchDeviceID(int reverseLimitSwitchDeviceID) {
        this.motorConfigs.reverseLimitSwitchDeviceID = reverseLimitSwitchDeviceID;
    }

    public void setForwardLimitSwitchNormal(LimitSwitchNormal forwardLimitSwitchNormal) {
        this.motorConfigs.forwardLimitSwitchNormal = forwardLimitSwitchNormal;
    }

    public void setReverseLimitSwitchNormal(LimitSwitchNormal reverseLimitSwitchNormal) {
        this.motorConfigs.reverseLimitSwitchNormal = reverseLimitSwitchNormal;
    }

    public void setSum0Term(FeedbackDevice sum0Term) {
        this.motorConfigs.sum0Term = sum0Term;
    }

    public void setSum1Term(FeedbackDevice sum1Term) {
        this.motorConfigs.sum1Term = sum1Term;
    }

    public void setDiff0Term(FeedbackDevice diff0Term) {
        this.motorConfigs.diff0Term = diff0Term;
    }

    public void setDiff1Term(FeedbackDevice diff1Term) {
        this.motorConfigs.diff1Term = diff1Term;
    }

}