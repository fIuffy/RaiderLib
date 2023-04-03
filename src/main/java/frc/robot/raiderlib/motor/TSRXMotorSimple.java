package frc.robot.raiderlib.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.raiderlib.motor.struct.MotorSimple;

public class TSRXMotorSimple extends MotorSimple{

    private final TalonSRX motor;
    private boolean isBrake;

    public TSRXMotorSimple(int canID, boolean brushless, String fileName, double minDutyCycle, boolean velocityControl, double maxOut) {
        super(canID, brushless, fileName, minDutyCycle, velocityControl, maxOut);
        this.motor = new TalonSRX(canID);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        this.isBrake = false;
    }

    @Override
    public void useAbsoluteForFeedback(boolean absolute) {
        motor.configSelectedFeedbackSensor(absolute ? FeedbackDevice.QuadEncoder : FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void setMotorControlPercent(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
        super.setMotorControlPercent(speed);
    }

    @Override
    public void setMotorPositional(double position) {
        motor.set(ControlMode.Position, position);
        super.setMotorPositional(position);
    }

    @Override
    public void setMotorVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity);
        super.setMotorVelocity(velocity);
    }

    @Override
    public void setEncoderPosition(double positon) {
        motor.setSelectedSensorPosition(positon);
    }

    @Override
    public void resetMotorPosition() {
        motor.setSelectedSensorPosition(0.0d);
    }

    @Override
    public void setPID(double pGain, double iGain, double dGain) {
        motor.config_kP(0, pGain);
        motor.config_kI(0, iGain);
        motor.config_kD(0, dGain);
    }

    @Override
    public void setPIDF(double pGain, double iGain, double dGain, double ffGain) {
        motor.config_kP(0, pGain);
        motor.config_kI(0, iGain);
        motor.config_kD(0, dGain);
        motor.config_kF(0, ffGain);
    }

    @Override
    public void setPIDTolerance(double tolerance) {
        motor.configAllowableClosedloopError(0, tolerance, 0);
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    public void setBrakeMode() {
        isBrake = true;
        motor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setCoastMode() {
        isBrake = false;
        motor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public double getMotorPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    public double getMotorVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    public double getMotorCurrent() {
        return motor.getStatorCurrent();
    }

    @Override
    public double getMotorOutputPercent() {
        return motor.getMotorOutputPercent();
    }

    @Override
    public boolean isBrake() {
        return isBrake;
    }

    @Override
    public boolean isCoast() {
        return !isBrake;
    }
    
}
