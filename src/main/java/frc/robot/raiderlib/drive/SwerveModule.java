package frc.robot.raiderlib.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.raiderlib.motor.TFXMotorSimple;

/**
 * SwerveModule class used to construct each individual module in a more simple matter.
 * @author Rob-Heslin and Chloe Quinn
 */
public class SwerveModule {
    public TFXMotorSimple driveMotor;
    public TFXMotorSimple rotateMotor;
    public CANCoder rotateSensor;

    public SwerveModule(int driveMotorID, int rotationMotorID, int canCoderID, boolean isInverted) {
        this.driveMotor = new TFXMotorSimple(driveMotorID, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE, true, DriveConstants.MAX_CONTROLPERCENT);
        configureDriveFeedback(isInverted);

        this.rotateMotor = new TFXMotorSimple(rotationMotorID, true, "module_rot", DriveConstants.MIN_SWERVE_ROTMOTOR_DUTYCYCLE, false, 1.0d);
        configureRotateMotorFeedback(isInverted);

        this.rotateSensor = new CANCoder(canCoderID);
        rotateSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        rotateSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
        rotateSensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
    }

     /**
     * Set the speed of the drive motor in percent duty cycle
     * 
     * @param dutyCycle a number between -1.0 and 1.0, where 0.0 is not moving, as
     *                  percent duty cycle
     */
    public void setDriveMotor(double dutyCycle) {
        driveMotor.setMotorControlPercent(dutyCycle);
    }

    public void setDriveMotorVelocity(double velocity) {
        driveMotor.setMotorVelocity(velocity);
    }

    /**
     * Set the speed of the rotation motor in percent duty cycle
     * 
     * @param dutyCycle a number between -1.0 and 1.0, where 0.0 is not moving, as
     *                  percent duty cycle
     */
    public void setRotationMotor(double dutyCycle) {
        rotateMotor.setMotorControlPercent(dutyCycle);
    }

    public void setRotationMotorPosition(double encoderPosition) {
        rotateMotor.setMotorPositional(encoderPosition);
    }

    

   /**
     * Sets drive motor to brake or coast
     * 
     * @param neutral whether to brake or coast   
     */
    public void setDriveMotorMode(NeutralMode neutral){
        driveMotor.getTalon().setNeutralMode(neutral);
    }

    /**
     * @return the distance the drive wheel has traveled (in meters)
     */
    public double getDriveDistance() {
        return driveMotor.getTalon().getSensorCollection().getIntegratedSensorPosition();
    }

    /**
     * Returns the speed of the drive wheel in Meters per second
     * 
     * @return speed of the drive wheel
     */
    public double getDriveVelocity() {
        return driveMotor.getTalon().getSensorCollection().getIntegratedSensorVelocity();
    }

    /**
     * Returns the speed of the drive wheel in Percent
     * 
     * @return input of the drive wheel
     */
    public double getDriveInput() {
        return driveMotor.getTalon().getMotorOutputPercent();
    }

    /**
     * A method to set the position of the drive encoder to zero,
     * essentially resetting it. 
     */
    public void resetDriveMotorEncoder() {
        driveMotor.resetMotorPosition();
    }

    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to set the offset of the 
     * CANCoder so we can dictate the zero position. 
     * INPUTS MUST BE IN DEGREES. 
     * 
     * @param value a number between -180 and 180, where 0 is straight ahead
     */
    private void setRotateAbsSensor(double value) {
        rotateSensor.configMagnetOffset(value, 0);
    }

    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to change the offset of 
     * the CANCoder so we dictate the zero position as the 
     * current position of the module.
     */
    public void zeroAbsPositionSensor() {
        //find the current offset, subtract the current position, and makes this number the new offset.
        setRotateAbsSensor(this.rotateSensor.configGetMagnetOffset()-getAbsPosInDeg());
    }

    /**
     * The CANCoder reads the absolute rotational position
     * of the module. This method returns that positon in 
     * degrees.
     * note: NOT Inverted module safe (use getPosInRad())
     * 
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getAbsPosInDeg() {
        return rotateSensor.getAbsolutePosition();
    }

    /**
     * sets the drive motor's PIDF for the PIDF controller on the TalonFX
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public void setDriveMotorPIDF(double P, double I, double D, double F) {
        driveMotor.getTalon().config_kP(0, P);
        driveMotor.getTalon().config_kI(0, I);
        driveMotor.getTalon().config_kD(0, D);
        driveMotor.getTalon().config_kF(0, F);
    }

    /**
     * sets the rotation motor's PIDF for the PIDF controller on the TalonFX
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public void setRotationMotorPIDF(double P, double I, double D, double F) {
        rotateMotor.getTalon().config_kP(0, P);
        rotateMotor.getTalon().config_kI(0, I);
        rotateMotor.getTalon().config_kD(0, D);
        rotateMotor.getTalon().config_kF(0, F);
    }

    /**
     * Returns the current angle of the swerve module, 
     * as read by the absolute rotational sensor, as a 
     * Rotation2d object. This is measured from the 
     * front of the robot, where counter-clockwise is 
     * positive.
     * 
     * @return A Rotation2d object, current position of the module
     */
    public Rotation2d getCurRot2d(){
        return Rotation2d.fromDegrees(getAbsPosInDeg());
    }

    /**
     * This method gets the current position in radians and 
     * normally the zero is at the front of the robot.
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad() {
        //get the current position and convert it to radians.
        return Math.toRadians(getAbsPosInDeg());
    }
   
    /**
     * Returns the current state of the swerve module 
     * as a SwerveModuleState. The speed of the module 
     * should be in m/s and the rotational position is 
     * in the form of a Rotation2d object.
     * 
     * @return a SwerveModuleState
     */
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), getCurRot2d());
    }

    /**
     * Returns the current position of the swerve module 
     * as a SwerveModulePosition. The position of the module 
     * should be in meters and the rotational position is 
     * in the form of a Rotation2d object.
     * 
     * @return a SwerveModulePosition
     */
    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getDriveDistance(), getCurRot2d());
    }

    public void resetRotationSensorPosition() {
        rotateMotor.resetMotorPosition();
    }

    /**
     * This is a method meant for testing by getting the count from the 
     * rotational encoder which is internal to the TalonFX. This encoder 
     * is relative, and does not easily translate to a specific rotational 
     * position of the swerve module.
     * 
     * @return the encoder count(no units, naturally just the count)
     */
    public double getRelEncCount() {
        return rotateMotor.getTalon().getSelectedSensorPosition()*DriveConstants.RAD_TO_ENC_FACTOR;
    }
    
    /**
     * The method to set the module to a position and speed. 
     * This method does the opitimization internally. The 
     * speed should be from -1.0 to 1.0 if isVeloMode is false, 
     * and should be between -MAX_VELOCITY and MAX_VELOCITY if 
     * isVeloMode is true.
     * 
     * @param targetState SwerveModuleState
     */
    public void setModuleState(SwerveModuleState targetState){
        
        // Instatiate Rotation2d object and fill with call from getCurRot2d()
        Rotation2d curPosition = getCurRot2d();
        
        // Optimize targetState with Rotation2d object pulled from above
        targetState = optimize(targetState, curPosition);
        
        // System.out.println("curAngle: "+curPosition.getDegrees()+"\t\t\t tarAngle: "+targetState.angle.getDegrees());

        // Find the difference between the target and current position
        double posDiff = targetState.angle.getRadians() - curPosition.getRadians(); 
        double absDiff = Math.abs(posDiff);
        
        // if the distance is more than a half circle, we are going the wrong way
        if (absDiff > Math.PI) {
            // the distance the other way around the circle
            posDiff = posDiff - (DriveConstants.TWO_PI * Math.signum(posDiff));
        }
        
        // Convert the shortest distance of rotation to relative encoder value(use convertion factor)
        double targetAngle = posDiff*DriveConstants.RAD_TO_ENC_FACTOR;
        // add the encoder distance to the current encoder count
        double outputEncValue = targetAngle + getRelEncCount();

        if(Math.abs(targetState.speedMetersPerSecond) < DriveConstants.MIN_DRIVE_VELOCITY) {
            stopAll();
        } else {
            // Set the setpoint using setReference on the TalonFX
            setRotationMotorPosition(outputEncValue);

            setDriveMotorVelocity(targetState.speedMetersPerSecond);
        }
    }

    public void setModuleStateDutyCycle(SwerveModuleState targetState){
        
        // Instatiate Rotation2d object and fill with call from getCurRot2d()
        Rotation2d curPosition = getCurRot2d();
        
        // Optimize targetState with Rotation2d object pulled from above
        targetState = optimize(targetState, curPosition);
        
        // System.out.println("curAngle: "+curPosition.getDegrees()+"\t\t\t tarAngle: "+targetState.angle.getDegrees());

        // Find the difference between the target and current position
        double posDiff = targetState.angle.getRadians() - curPosition.getRadians(); 
        double absDiff = Math.abs(posDiff);
        
        // if the distance is more than a half circle, we are going the wrong way
        if (absDiff > Math.PI) {
            // the distance the other way around the circle
            posDiff = posDiff - (DriveConstants.TWO_PI * Math.signum(posDiff));
        }
        
        // Convert the shortest distance of rotation to relative encoder value(use convertion factor)
        double targetAngle = posDiff*DriveConstants.RAD_TO_ENC_FACTOR;
        // add the encoder distance to the current encoder count
        double outputEncValue = targetAngle + getRelEncCount();

        if(Math.abs(targetState.speedMetersPerSecond) < DriveConstants.MIN_DRIVE_DUTYCYCLE) {
            stopAll();
        } else {
            // Set the setpoint using setReference on the TalonFX
            setRotationMotorPosition(outputEncValue);

            setDriveMotor(targetState.speedMetersPerSecond);
        }
    }

    /**
     * The method to set the module to a position and speed. 
     * This method does the opitimization internally. The 
     * speed should be from -1.0 to 1.0 if isVeloMode is false, 
     * and should be between -MAX_VELOCITY and MAX_VELOCITY if 
     * isVeloMode is true.
     * 
     * @param targetAngle Angle to rotate module to (in degrees)
     */
    public void setModuleStateRotKeepSpeed(double desiredAngle){
        
        // Instatiate Rotation2d object and fill with call from getCurRot2d()
        Rotation2d curPosition = getCurRot2d();
        
        // Optimize targetState with Rotation2d object pulled from above
        SwerveModuleState targetState = optimize(new SwerveModuleState(getModuleState().speedMetersPerSecond, Rotation2d.fromDegrees(desiredAngle)), curPosition);

        // Find the difference between the target and current position
        double posDiff = targetState.angle.getRadians() - curPosition.getRadians(); 
        double absDiff = Math.abs(posDiff);
        
        // if the distance is more than a half circle, we are going the wrong way
        if (absDiff > Math.PI) {
            // the distance the other way around the circle
            posDiff = posDiff - (DriveConstants.TWO_PI * Math.signum(posDiff));
        }
        
        // Convert the shortest distance of rotation to relative encoder value(use convertion factor)
        double targetAngle = posDiff * DriveConstants.RAD_TO_ENC_FACTOR;
        // add the encoder distance to the current encoder count
        double outputEncValue = targetAngle + getRelEncCount();

        setRotationMotorPosition(outputEncValue);
        setDriveMotorVelocity(targetState.speedMetersPerSecond);
    }

    /**
     * This method is used to stop the module completely. The drive 
     * motor is switched to percent voltage and and output of 0.0 
     * percent volts. The rotation motor's PIDController is set to 
     * DutyCyclevoltage control mode, and output of 0.0% output.
     */
    public void stopAll() {
        driveMotor.getTalon().set(TalonFXControlMode.PercentOutput, 0.0);
        rotateMotor.getTalon().set(TalonFXControlMode.PercentOutput, 0.0);
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getRadians()) > (Math.PI/2)) {
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Rotation2d.fromDegrees(180)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }


    private void configureDriveFeedback(boolean isInverted) {
        TalonFX driveTalon = driveMotor.getTalon();
        driveTalon.configFactoryDefault();
        // use the integrated sensor with the primary closed loop and timeout is 0.
        driveTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        driveTalon.configSelectedFeedbackCoefficient(DriveConstants.ENC_TO_METERS_FACTOR);
        // above uses configSelectedFeedbackCoefficient(), to scale the
        // driveMotor to real distance, DRIVE_ENC_TO_METERS_FACTOR
        driveTalon.setNeutralMode(NeutralMode.Brake);
        driveTalon.setInverted(isInverted);// Set motor inverted(set to false)
        driveTalon.enableVoltageCompensation(true);
        driveTalon.configVoltageCompSaturation(DriveConstants.MAX_VOLTAGE);

        driveTalon.config_kP(0, DriveConstants.ROBOT_TRANSLATION_P);
        driveTalon.config_kI(0, DriveConstants.ROBOT_TRANSLATION_I);
        driveTalon.config_kD(0, DriveConstants.ROBOT_TRANSLATION_D);
        driveTalon.config_kF(0, DriveConstants.ROBOT_TRANSLATION_FF);

        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 253);
        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);//This is key to odometry must be around same as code loop
        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 251);
        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 239);
        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 229);
        driveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
        driveTalon.setSelectedSensorPosition(0.0);
    }
    
    private void configureRotateMotorFeedback(boolean isInverted) {
        TalonFX rotateTalon = rotateMotor.getTalon();
        rotateTalon.configFactoryDefault();
        // use the integrated sensor with the primary closed loop and timeout is 0.
        rotateTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        rotateTalon.configSelectedFeedbackCoefficient(1/DriveConstants.RAD_TO_ENC_FACTOR);
        rotateTalon.setNeutralMode(NeutralMode.Brake);
        rotateTalon.setInverted(true);
        rotateTalon.enableVoltageCompensation(true);
        rotateTalon.configVoltageCompSaturation(DriveConstants.MAX_VOLTAGE);
        

        rotateTalon.config_kP(0, DriveConstants.SWERVE_ROT_P);
        rotateTalon.config_kI(0, DriveConstants.SWERVE_ROT_I);
        rotateTalon.config_kD(0, DriveConstants.SWERVE_ROT_D);
        rotateTalon.config_kF(0, DriveConstants.SWERVE_ROT_FF);

        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 240);//This packet is the motor output, limit switches, faults, we care about none of those
        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);//This is the sensor feedback, i.e. relative encoder
        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 251);
        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 239);
        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 229);
        rotateTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
        rotateTalon.setSelectedSensorPosition(0.0);
        rotateTalon.configAllowableClosedloopError(0, DriveConstants.SWERVE_MODULE_TOLERANCE, 0);
    }
}
