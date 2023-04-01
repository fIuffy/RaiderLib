package frc.robot.raiderlib.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.raiderlib.motor.struct.MotorSimple;

public class SPARKMotorSimple extends MotorSimple{

    private final CANSparkMax motor;

    public SPARKMotorSimple(int canID, boolean brushless, String fileName, double minDutyCycle, boolean velocityControl, double maxOut) {
        super(canID, brushless, fileName, minDutyCycle, velocityControl, maxOut);
        this.motor = new CANSparkMax(canID, brushless ? MotorType.kBrushless : MotorType.kBrushed);
        /**
         * PeriodicFrame.kStatus2 contains just encoder position,
         * so we set the frame period to 20ms as that is how WPILIB PIDControllers
         * are running.
         */
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    }

    @Override
    public void useAbsoluteForFeedback(boolean absolute) {
        motor.getPIDController().setFeedbackDevice(!absolute ? motor.getEncoder() : motor.getAbsoluteEncoder(Type.kDutyCycle));
    }

    @Override
    public void setMotorControlPercent(double speed) {
        motor.set(speed);
        super.setMotorControlPercent(speed);
    }

    @Override
    public void setMotorPositional(double position) {
        motor.getPIDController().setReference(position, ControlType.kPosition);
        super.setMotorPositional(position);
    }

    @Override
    public void setMotorVelocity(double velocity) {
        motor.getPIDController().setReference(velocity, ControlType.kVelocity);
        super.setMotorVelocity(velocity);
    }

    @Override
    public void resetMotorPosition() {
        motor.getEncoder().setPosition(0.0d);
    }

    @Override
    public void setPID(double pGain, double iGain, double dGain) {
        motor.getPIDController().setP(pGain);
        motor.getPIDController().setI(iGain);
        motor.getPIDController().setD(dGain);
    }

    @Override
    public void setPIDF(double pGain, double iGain, double dGain, double ffGain) {
        motor.getPIDController().setP(pGain);
        motor.getPIDController().setI(iGain);
        motor.getPIDController().setD(dGain);
        motor.getPIDController().setFF(ffGain);
    }

    @Override
    public void setPIDTolerance(double tolerance) {
        motor.getPIDController().setSmartMotionAllowedClosedLoopError(tolerance, 0);
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    public void setBrakeMode() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setCoastMode() {
        motor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public double getMotorPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getMotorVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getMotorOutputPercent() {
        return motor.get();
    }

    @Override
    public boolean isBrake() {
        return motor.getIdleMode() == IdleMode.kBrake;
    }

    @Override
    public boolean isCoast() {
        return motor.getIdleMode() == IdleMode.kCoast;
    }
    
}
