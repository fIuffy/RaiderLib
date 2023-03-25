package frc.robot.raiderlib.drive.struct;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveSystem {
    private final PIDController spinController;
    private final XboxController controller;
    private Pose2d currentPose;
    private final AHRS gyro;
    private DriveMode driveMode;

    public boolean exporting = false;
    public String exportFile = "/home/lvuser/pid_exports/drive/";
    public BufferedWriter writer;
    public long startTime = -1L;

    public double exportStepTime = 1.5d;

    public DriveSystem(XboxController controller, String exportName) {
        this.controller = controller;
        this.currentPose = new Pose2d();
        this.gyro = new AHRS(SerialPort.Port.kUSB);
        this.spinController = new PIDController(DriveConstants.ROBOT_ROT_P, DriveConstants.ROBOT_ROT_I, DriveConstants.ROBOT_ROT_D);
        this.exporting = false;
        this.exportFile+=exportName;
        this.driveMode = DriveMode.ROBOT_CENTRIC;
    }


    /**
     * Super constructor must be called! (Handles DriveMode (field vs robot) as well as PIDExports)
     * Override for odometry
     */
    public void periodic() {
        if(this.exporting) {
            exportPeriodic();
        }
        switch(driveMode) {
            case ROBOT_CENTRIC: this.driveRobotCentricMethod(null); break;
            case FIELD_CENTRIC: this.driveFieldCentricMethod(null); break;
        }
    }

    /**
     * Drive relative to the robot
     */
    public void driveRobotCentricMethod(ChassisSpeeds speeds) {

    }

    /**
     * Drive relative to the field
     */
    public void driveFieldCentricMethod(ChassisSpeeds speeds) {

    }

    /**
     * Change the DriveMode
     * @param driveMode - DriveMode enum (ROBOT_CENTRIC or FIELD_CENTRIC)
     */
    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    /**
     * 
     * @return - Current Position (Pose2d)
     */
    public Pose2d getCurrentPose() {
        return this.currentPose;
    }

    /**
     * Sets the robot's current position
     * @param newPose - New position
     */
    public void setCurrentPose(Pose2d newPose) {
        this.currentPose = newPose;
    }

    public XboxController getController() {
        return this.controller;
    }

    public AHRS getGyro() {
        return this.gyro;
    }

    public PIDController getSpinController() {
        return this.spinController;
    }

    public DriveMode getDriveMode() {
        return this.driveMode;
    }

    /**
     * Enable PID Exporting (creates a file).
     * I (Chloe) personally recommend to use an online PIDTuner to make tuning less of a trial and error thing.
     * Although using exported PID data is not required, I have created an automated export process incase it is desired.
     * @see https://pidtuner.com
     */
    public void enablePIDExport() {
        try { 
            new File("/home/lvuser/pid_exports").mkdir();
            new File("/home/lvuser/pid_exports/drive").mkdir();
            new File(exportFile).createNewFile();
            writer = new BufferedWriter(new FileWriter(exportFile)); 
        } catch(Exception e){ 
            e.printStackTrace(); 
        }
        this.startTime = System.currentTimeMillis();
        this.exporting = true;
    }

    /**
     * Enabling PIDExport of motor to be ran as a command.
     * @return Command runnable by WPILib's Command-based Structure
     */
    public Command exportPIDData() {
        return Commands.runOnce(() -> this.enablePIDExport());
    }

    /**
     * PIDExport ran periodically
     */
    public void exportPeriodic() {

    }

    /**
     * Disable PID Exporting.
     */
    public void disablePIDExport() {
        try {
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        this.startTime = -1L;
        this.exporting = false;
    }

    /**
     * A method to return the value of a driver joystick axis,
     * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
     * value returned if the joystick value is between -.1 and 
     * .1)
     * @param axis
     * @return value of the joystick, from -1.0 to 1.0 where 0.0 is centered
     * @author Rob Heslin
     */
    public double getDriverAxis(Axis axis, XboxController driver) {
        return (driver.getRawAxis(axis.value) < -.1 || driver.getRawAxis(axis.value) > .1)
                ? driver.getRawAxis(axis.value)
                : 0.0;
    }

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }


}
