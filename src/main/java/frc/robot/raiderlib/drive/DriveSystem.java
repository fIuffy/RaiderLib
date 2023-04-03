package frc.robot.raiderlib.drive;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveSystem {
    public final PIDController spinController;
    private final XboxController controller;
    private Pose2d currentPose;
    private final AHRS gyro;
    private DriveMode driveMode;

    public boolean exporting = false;
    public String exportFile = "/home/lvuser/pid_exports/drive/";
    public BufferedWriter writer;
    public long startTime = -1L;

    public double exportStepTime = 1.5d;

    public boolean spinExporting;
    public boolean poseExporting;
    public double currentAppliedInput;

    public DriveSystem(XboxController controller, String exportName) {
        this.controller = controller;
        this.currentPose = new Pose2d();
        this.gyro = new AHRS(SerialPort.Port.kUSB);
        this.spinController = new PIDController(DriveConstants.ROBOT_ROT_P, DriveConstants.ROBOT_ROT_I, DriveConstants.ROBOT_ROT_D);
        this.spinController.setTolerance(DriveConstants.ROBOT_ROT_TOLERANCE);
        this.exporting = false;
        this.exportFile = this.exportFile+exportName+".csv";
        this.driveMode = DriveMode.ROBOT_CENTRIC;

        spinExporting = false;
        poseExporting = false;
        currentAppliedInput = 0.0d;

        DriveConstants.addPathEventMarkers();
    }
    
    /**
     * Reset the gyro.
     */
    public void resetGyro(){
        getGyro().reset();
    }

    /**
     * Change the current gyro angle
     * @param newCurrentAngle New angle in degrees
     */
    public void setGyro(double newCurrentAngle){
        resetGyro();
        getGyro().setAngleAdjustment(newCurrentAngle);
    }


    /**
     * 
     * @return current Gyro angle in Rotation2d object.
     */
    public Rotation2d getGyroRot2d(){
        return Rotation2d.fromDegrees(getGyroInDeg());
    }

    /**
     * 
     * @return current Gyro angle in radians
     */
    public double getGyroInRad(){
        return Math.toRadians(getGyroInDeg());
    }

    /**
     * 
     * @return current Gyro angle in degrees
     */
    public double getGyroInDeg(){
        return getGyro().getAngle()*-1;
    }

    /**
     * 
     * @return Get the rate of yaw rotation from the gyro in degrees per second.
     */
    public double getRotationalVelocity(){
        return getGyro().getRate()*-1;
    }
    
    /**
      * Will make PIDExporting drive and export RotateSpeed as opposed to X speed.
      */
      public void enableSpinExporting() {
        this.spinExporting = true;
      }
  
      /**
       * Disable spin exporting mode.
       */
      public void disableSpinExporting() {
        this.spinExporting = false;
      }

    /**
      * Will make PIDExporting drive and export Velocity (m/s) vs x (meters)
      */
      public void enablePoseExporting() {
        this.poseExporting = true;
      }
  
      /**
       * Disable spin exporting mode.
       */
      public void disablePoseExporting() {
        this.poseExporting = false;
      }
    

    /**
     * Override for odometry.
     * It is recommended that the super constructor be called as it enables the handling of XboxController-based driving as well as PIDExports.
     * XboxController-based driving is disabled while in Autonomous to not screw up possible Autonomous drive handling.
     */
    public void periodic() {
        if(this.exporting) {
            exportPeriodic();
        }
        // Do not run the joystick null input when we are in Autonomous or if we are exporting data!
        if(RobotState.isAutonomous() || this.exporting) return;
        switch(driveMode) {
            case ROBOT_CENTRIC: this.driveRobotCentricMethod(null); break;
            case FIELD_CENTRIC: this.driveFieldCentricMethod(null); break;
        }
    }

    /**
     * Controller driving relative to the robot ran periodically when DriveMode is ROBOT_CENTRIC
     */
    public void driveRobotCentricMethod(ChassisSpeeds speeds) {

    }

    /**
     * Controller driving relative to the robot ran periodically when DriveMode is FIELD_CENTRIC
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
     * Method to reset the DriveTrain's sensors in a standardized fashion.
     */
    public void zeroDriveSensors() {

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
     * @see https://pidtuner.com++
++    */
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
     * @param driveExportMode DriveExportMode enum
     * @return Command runnable by WPILib's Command-based Structure
     */
    public Command exportPIDData(DriveExportMode driveExportMode) {
        switch(driveExportMode) {
            case VELOCITY:
                disablePoseExporting();
                disableSpinExporting();
                break;
            case SPIN:
                disablePoseExporting();
                enableSpinExporting();
            case POSITION:
                enablePoseExporting();
                disableSpinExporting();
            case MODULESPIN:
                disablePIDExport();
                return exportSingleModuleRot();
            default:
                break;
            
        }
        return Commands.runOnce(() -> this.enablePIDExport());
    }

    public enum DriveExportMode {
        VELOCITY,
        SPIN,
        POSITION,
        MODULESPIN
    }

    /**
     * Zeroing the DriveTrain in command form.
     * @return Command runnable by WPILIB's Command-based Structure
     */ 
    public Command zeroDriveTrain() {
        return Commands.runOnce(() -> this.zeroDriveSensors());
    }

    /**
     * Convert Controller input to Holonomic (Swerve/Mecanum) ChassisSpeeds
     * @return ChassisSpeeds object
     */
    public ChassisSpeeds controllerToHolonomicSpeeds() {
        double moveLateralControlPercent = getDriverAxis(Axis.kLeftY, getController());
        double moveStrafeControlPercent = getDriverAxis(Axis.kLeftX, getController());
        double rotateControlPercent = getDriverAxis(Axis.kRightX, getController());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(moveLateralControlPercent*DriveConstants.MAX_VELOCITY, moveStrafeControlPercent*DriveConstants.MAX_STRAFE_VELOCITY, rotateControlPercent*DriveConstants.MAX_ANGULAR_VELOCITY);

        return chassisSpeeds;
    }

    /**
     * Convert Controller input to Differential ChassisSpeeds (no Y movement)
     * @return ChassisSpeeds object
     */
    public ChassisSpeeds controllerToDifferentialSpeeds() {
        double moveLateralControlPercent = getDriverAxis(Axis.kLeftY, getController());
        double rotateControlPercent = getDriverAxis(Axis.kRightX, getController());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(moveLateralControlPercent*DriveConstants.MAX_VELOCITY, 0.0d, rotateControlPercent*DriveConstants.MAX_ANGULAR_VELOCITY);

        return chassisSpeeds;
    }

    /**
     * Method to drive forward and back (or use rotate speed if we are spinExporting is enabled).
     * @param input ControlPercent speed (-1.0 to 1.0 with 0.0 being at rest)
     */
    public void driveAllControlPercent(double input) {

    }

    public Command fullAuto(String pathName) {
        return new FollowPathWithEvents(
            followPathCommand(pathName, true),
            getPathMarkers(pathName),
            DriveConstants.markerMap
        );
    }

    public List<EventMarker> getPathMarkers(String pathName) {
        return PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName)).getMarkers();
    }

    /**
     * Returns a command that follows a PathPlanner path by path name using the drive train's base methods
     * @param pathName - Name of the path
     * @param resetOdometry - Usually true if this is the first path being followed
     * @return Runnable command that follows a PathPlanner path by name.
     */
    public Command followPathCommand(String pathName, boolean resetOdometry) {
        return followTrajectoryCommand(PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName)), resetOdometry);
    }

    /**
     * Command that when executed, follows a given PathPlannerTrajectory using the drivetain's
     * base methods.
     * 
     * @param traj - PathPlannerTrajectory
     * @param isFirstPath - Resets the odometry if true
     * @return Runnable tajectory following command
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return null;
    }

    public void resetOdometry(Pose2d pose) {
        
    }

    public void enableSingleModuleExport() {
        
    }

    public Command exportSingleModuleRot() {
        return Commands.runOnce(() -> enableSingleModuleExport());
    }

    /**
     * A DriveSystems PIDExport ran periodically while exporting field is true
     */
    public void exportPeriodic() {
        long difference = System.currentTimeMillis() - startTime;
        BigDecimal bd = BigDecimal.valueOf(difference).movePointLeft(3);
        double diffSeconds = bd.doubleValue();
        double factorSpeed = (this.spinExporting) ? DriveConstants.MIN_ROTSPEED_DUTYCYCLE : DriveConstants.MIN_DRIVE_DUTYCYCLE;
        switch((int)Math.floor(diffSeconds/this.exportStepTime)) {
            case 0: this.driveAllControlPercent(0.0d); resetGyro(); break;
            case 1: this.driveAllControlPercent(factorSpeed); break;
            case 2: this.driveAllControlPercent(0.0d); break;
            case 3: this.driveAllControlPercent(-factorSpeed); break;
            case 4: this.driveAllControlPercent(0.0d); break;
            case 5: this.driveAllControlPercent(factorSpeed*2); break;
            case 6: this.driveAllControlPercent(0.0d); break;
            case 7: this.driveAllControlPercent(-factorSpeed*2); break;
            case 8: this.driveAllControlPercent(0.0d); break;
            default: this.disablePIDExport(); return;
        }
        /**
         * Actually append to the FileWriter
         * We use append so that we can just simply add on without needed to write to the file through one object.
         */
        try {
            String writeString = diffSeconds+","+this.currentAppliedInput+","+(getAvgDriveVelociy());

            double rotationalVeloInRad = getRotationalVelocity() * (Math.PI/180d);
            if(spinExporting) writeString = diffSeconds+","+rotationalVeloInRad+","+(getGyroInRad());

            if(poseExporting) writeString = diffSeconds+","+getAvgDriveVelociy()+","+(getCurrentPose().getX());
            writeString+="\n";
            writer.append(writeString);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Return an average velocity (m/s) based on all motor speeds
     * @return Velocity (m/s)
     */
    public double getAvgDriveVelociy() {
        return 0.0d;
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
     * @author Rob
     */
    public double getDriverAxis(Axis axis, XboxController driver) {
        return (driver.getRawAxis(axis.value) < -.1 || driver.getRawAxis(axis.value) > .1)
                ? driver.getRawAxis(axis.value)
                : 0.0;
    }

    /**
     * Basic DriveMode enum to make coding more like English
     * Or as Mr. Schlegel puts it: "Speaking pseudocode"
     */
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    /**
     * Use spin PIDController to convert an angle to robot rotateSpeed.
     * @param target - Angle
     * @param radians - If true, accepts the target angle parameter as radians
     * @return rotateSpeed to be used in ChassisSpeeds in radians per second
     */
    public double angleToRotationSpeed(double target, boolean radians){
        double currentGyroPos = getGyroInRad();

        if(!radians) target=Math.toRadians(target);
        double output = spinController.calculate(currentGyroPos, target);

        if(spinController.atSetpoint()) return 0.0d;
        if (Math.abs(output) < DriveConstants.MIN_ROTSPEED){
            return DriveConstants.MIN_ROTSPEED*Math.signum(output);
        }
        return output;
    }


}
