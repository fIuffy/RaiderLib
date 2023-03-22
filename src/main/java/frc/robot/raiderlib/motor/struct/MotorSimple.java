package frc.robot.raiderlib.motor.struct;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MotorSimple {

    public boolean exporting = false;
    public String exportFile = "/home/lvuser/pid_exports/";
    public BufferedWriter writer;
    public long startTime = -1L;

    public double exportStepTime = 1.5d;

    public double maxOut;

    public double minDutyCycle;
    public boolean velocityControl;

    /**
     * MotorSimple constructor which makes creating Motors easier as different companies make different controllers and different libraries.
     * So this is made in order to make it Standard to confuse newer 1518 programmers.
     * @param canID - ID on the CANBus.
     * @param brushless - Whether motor is brushless or not.
     * @param fileName - FileName for PID Exports.
     * @param minDutyCycle - Minimum speed in ControlPercent format.
     * @param velocityControl - Use velocity or not (Only really needed for things that drive wheels).
     * @param maxOut - Limiter for ControlPercent input.
     */
    public MotorSimple(int canID, boolean brushless, String fileName, double minDutyCycle, boolean velocityControl, double maxOut) {
        this.exportFile+=fileName;
        this.minDutyCycle = minDutyCycle;
        this.velocityControl = velocityControl;
        this.maxOut = maxOut;
    }
    

    /**
     * Set the feedback to be based on whether we are used an AbsoluteEncoder attached
     * to the controller or not
     * @param absolute - Use the absolute encoder
     */
    public void setFeedback(boolean absolute) {

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
     * Called to limit our ControlPercent speed to a maximum decimal 0.0-1.0
     */
    public void maxSpeedCheck() {
        if(this.getMotorOutputPercent() > maxOut) this.setMotorControlPercent(maxOut);
    }

    /**
     * Enable PID Exporting, creates a file.
     * Note: I (Chloe) personally recommend to use an online PIDTuner to make tuning less of a trial and error thing.
     * Although using exported PID data is not required, I have created an automated export process incase it is desired.
     * @see https://pidtuner.com
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
}
