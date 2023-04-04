package me.chloe.raiderlib.motor;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import me.chloe.raiderlib.builders.MotorControllerSimple;

/**
 * Base constructor that is a subset of a MotorControllerSimple
 */
public class MotorSimple {
    private final int canID;
    public boolean exporting = false;
    public String exportFile = "/home/lvuser/pid_exports/";
    public BufferedWriter writer;
    public long startTime = -1L;

    public double exportStepTime = 1.5d;

    public double maxOut;

    public double minDutyCycle;
    public boolean velocityControl;

    public double encoderConversionFactor = 1.0d;

    /**
     * Current state of the motor, contains position and velocity information.
     * Updates periodically.
     */
    public MotorSimpleState motorSimpleState;

    /**
     * MotorSimple constructor which makes creating Motors easier as different companies make different controllers and different libraries.
     * So this is made in order to make it standard to lessen confusion for newer 1518 programmers.
     * @param canID ID on the CANBus.
     * @param brushless Whether motor is brushless or not.
     * @param fileName FileName for PID Exports.
     * @param minDutyCycle Min limiter for speed input.
     * @param velocityControl Whether to use velocity or not for PIDExports.
     * @param maxOut Max limiter for speed input.
     */
    public MotorSimple(int canID, boolean brushless, String fileName, double minDutyCycle, boolean velocityControl, double maxOut) {
        this.canID = canID;
        this.exportFile = this.exportFile+fileName+".csv";
        this.minDutyCycle = minDutyCycle;
        this.velocityControl = velocityControl;
        this.maxOut = maxOut;
    }

    /**
     * Get the MotorSimpleState in periodic form.
     * @param simple MotorControllerSimple
     */
    public void setMotorStateController(MotorControllerSimple simple) {
        this.motorSimpleState = new MotorSimpleState(null, 0.0d, velocityControl);
    }

    /**
     * Get the motor's parent controller CANBus id.
     * @return (id) int
     */
    public int getCANID() {
        return this.canID;
    }


    /**
     * Set the MotorState to a target position/velocity
     * @param state MotorSimpleState
     */
    public void setMotorState(MotorSimpleState state) {
        this.setMotorPositional(state.statePosition);
        if(state.stateSpeed != 0.0d) this.setMotorVelocity(state.stateSpeed);
    }

    /**
     * Set the Encoder Conversion Factor (Used for velocity/drive control)
     * @param factor - Value to set conversion factor to
     */
    public void setEncoderConversionFactor(double factor) {
        this.encoderConversionFactor = factor;
    }

    /**
     * Get the encoder positon times the conversion factor.
     * @return double
     */
    public double getMotorPositionConverted() {
        return this.getMotorPosition()*this.encoderConversionFactor;
    }

    /**
     * Get the encoder velocity times the conversion factor.
     * @return double
     */
    public double getMotorVelocityConverted() {
        return this.getMotorVelocity()*this.encoderConversionFactor;
    }
    

    /**
     * Set the feedback to be based on whether we are used an AbsoluteEncoder attached
     * to the controller or not
     * @param absolute - Use the absolute encoder
     */
    public void useAbsoluteForFeedback(boolean absolute) {

    }

    /**
     * Set motor speed in terms of ControlPercent
     * @param speed - Value -1.0 through 1.0
     */
    public void setMotorControlPercent(double speed) {
        maxSpeedCheck();
    }

    /**
     * Set motor position - MUST BE PID TUNED BEFORE USAGE
     * @param position - Encoder position
     */
    public void setMotorPositional(double position) {
        maxSpeedCheck();
    }

    /**
     * Set motor velocity - MUST BE PID TUNED BEFORE USAGE
     * @param velocity - Velocity in m/s
     */
    public void setMotorVelocity(double velocity) {
        maxSpeedCheck();
    }

    public void setEncoderPosition(double positon) {
        
    }

    /**
     * Set motor encoder position to 0
     */
    public void resetMotorPosition() {

    }

    /**
     * 
     * @param pGain - Proportional gain
     * @param iGain - Integral gain
     * @param dGain - Derivative gain
     */
    public void setPID(double pGain, double iGain, double dGain) {

    }

    /**
     * 
     * @param pGain - Proportional gain
     * @param iGain - Integral gain
     * @param dGain - Derivative gain
     * @param ffGain - Feed forward gain
     */
    public void setPIDF(double pGain, double iGain, double dGain, double ffGain) {

    }

    public void updateState(double position, double velocity) {
        motorSimpleState.setCurrentPosition(position);
        motorSimpleState.setCurrentSpeed(velocity);
    }

    public void setState(MotorSimpleState state) {
        this.motorSimpleState = state;
    }

    public MotorSimpleState getState() {
        return this.motorSimpleState;
    }

    /**
     * 
     * @param tolerance - PID Error tolerance
     */
    public void setPIDTolerance(double tolerance) {

    }

    /**
     * 
     * @param inverted Is motor inverted or not
     */
    public void setInverted(boolean inverted) {

    }

    /**
     * Set motor to Brake Mode
     */
    public void setBrakeMode() {

    }

    /**
     * Set motor to Coast Mode
     */
    public void setCoastMode() {

    }

    /**
     * Called to limit our ControlPercent speed to a maximum decimal -1.0 to 1.0
     */
    public void maxSpeedCheck() {
        if(Math.abs(this.getMotorOutputPercent()) > maxOut) this.setMotorControlPercent(this.getMotorOutputPercent() > 0 ? maxOut : -maxOut);
    }

    /**
     * Enable PID Exporting (creates a file).
     * I (Chloe) personally recommend to use an online PIDTuner to make tuning less of a trial and error thing.
     * Although using exported PID data is not required, I have created an automated export process incase it is desired.
     * Link to PIDTuner: https://pidtuner.com
     */
    public void enablePIDExport() {
        this.exporting = true;
        try { 
            new File("/home/lvuser/pid_exports").mkdir();
            new File(exportFile).createNewFile();
            writer = new BufferedWriter(new FileWriter(exportFile)); 
        } catch(Exception e){ 
            e.printStackTrace(); 
        }
        this.startTime = System.currentTimeMillis();
    }

    /**
     * Disable PID Exporting.
     */
    public void disablePIDExport() {
        this.exporting = false;
        try {
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        this.startTime = -1L;
    }


    /**
     * Basic periodic function to run 8 steps that write to a file on the RoboRIO.
     */
    public void exportPeriodic() {
        long difference = System.currentTimeMillis() - startTime;
        BigDecimal bd = BigDecimal.valueOf(difference).movePointLeft(3);
        double diffSeconds = bd.doubleValue();
        switch((int)Math.floor(diffSeconds/this.exportStepTime)) {
            case 0: this.setMotorControlPercent(0.0d); break;
            case 1: this.setMotorControlPercent(this.minDutyCycle); break;
            case 2: this.setMotorControlPercent(0.0d); break;
            case 3: this.setMotorControlPercent(-this.minDutyCycle); break;
            case 4: this.setMotorControlPercent(0.0d); break;
            case 5: this.setMotorControlPercent(this.minDutyCycle*2); break;
            case 6: this.setMotorControlPercent(0.0d); break;
            case 7: this.setMotorControlPercent(-this.minDutyCycle*2); break;
            case 8: this.setMotorControlPercent(0.0d); break;
            default: this.disablePIDExport(); return;
        }
        /**
         * Actually append to the FileWriter
         * We use append so that we can just simply add on without needed to write to the file through one object.
         */
        try {
            writer.append(diffSeconds+","+this.getMotorOutputPercent()+","+(velocityControl ? this.getMotorVelocity() : this.getMotorPosition()) + "\n");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Setting ControlPercent input of motor to be ran as a command.
     * @param percent - Value -1.0 through 1.0
     * @return Command runnable by WPILib's Command-based Structure
     */
    public Command setPercentOutputCommand(double percent) {
        return Commands.runOnce(() -> this.setMotorControlPercent(percent));
    }

    /**
     * Setting position of motor to be ran as a command.
     * @param position - Encoder position
     * @return Command runnable by WPILib's Command-based Structure
     */
    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> this.setMotorPositional(position));
    }

    /**
     * Setting velocity of motor to be ran as a command.
     * @param velocity - Velocity (m/s)
     * @return Command runnable by WPILib's Command-based Structure
     */
    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> this.setMotorPositional(velocity));
    }

    /**
     * Enabling PIDExport of motor to be ran as a command.
     * @return Command runnable by WPILib's Command-based Structure
     */
    public Command exportPIDData() {
        return Commands.runOnce(() -> this.enablePIDExport());
    }

    /**
     * 
     * @return Motor encoder position. (useAbsolute?)
     */
    public double getMotorPosition() {
        return 0.0d;
    }
    /**
     * 
     * @return Motor encoder-recorded velocity. (useAbsolute?)
     */
    public double getMotorVelocity() {
        return 0.0d;
    }
    /**
     * 
     * @return Motor input current. (Does not work on VictorSPX)
     */
    public double getMotorCurrent() {
        return 0.0d;
    }

    /**
     * 
     * @return Applied Motor input (-1.0 to 1.0)
     */
    public double getMotorOutputPercent() {
        return 0.0d;
    }

    /**
     * 
     * @return Is motor in Brake mode
     */
    public boolean isBrake() {
        return false;
    }

    /**
     * 
     * @return Is motor in Coast mode
     */
    public boolean isCoast() {
        return true;
    }




    public class MotorSimpleState {
        private final MotorControllerSimple motorControllerSimple;
        private double statePosition;
        private double stateSpeed;
        public boolean velocityControl;

        /**
         * Create a MotorSimpleState to correlate specific speeds by MotorControllerSimple
         * @param motorControllerSimple MotorControllerSimple of motor
         * @param stateSpeed double
         * @param velocityControl When updating states, if true, sets by velocity, if false, sets by ControlPercent
         */
        public MotorSimpleState(MotorControllerSimple motorControllerSimple, double stateSpeed, boolean velocityControl) {
            this.motorControllerSimple = motorControllerSimple;
            this.statePosition = -1;
            this.stateSpeed = stateSpeed;
            this.velocityControl = velocityControl;
        }

        /**
         * Create a MotorSimpleState to correlate specific positions by MotorControllerSimple
         * @param motorControllerSimple MotorControllerSimple of motor
         * @param statePosition double
         */
        public MotorSimpleState(MotorControllerSimple motorControllerSimple, double statePosition) {
            this.motorControllerSimple = motorControllerSimple;
            this.statePosition = statePosition;
            this.stateSpeed = -1;
            this.velocityControl = false;
        }

        /**
         * 
         * @return MotorControllerSimple
         */
        public MotorControllerSimple getController() {
            return this.motorControllerSimple;
        }

        /**
         * State's positon
         * @return double
         */
        public double getStatePosition() {
            return this.statePosition;
        }

        /**
         * State's speed
         * @return double
         */
        public double getStateSpeed() {
            return this.stateSpeed;
        }

        /**
         * Set state postion
         * @param position double
         */
        public void setCurrentPosition(double position) {
            this.statePosition = position;
        }

        /**
         * Set state speed
         * @param speed double
         */
        public void setCurrentSpeed(double speed) {
            this.stateSpeed = speed;
        }
    }
}
