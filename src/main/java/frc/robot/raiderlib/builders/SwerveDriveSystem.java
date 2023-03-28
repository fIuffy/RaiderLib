package frc.robot.raiderlib.builders;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.raiderlib.drive.DriveConstants;
import frc.robot.raiderlib.drive.DriveSystem;
import frc.robot.raiderlib.drive.SwerveModule;


/**
 * SwerveDriveSystem class used to construct a SwerveBase in a very simplistic way.
 * Automatically implements XboxController control on USB port 0.
 * Rewritten from GRR 340's 2022 SwerveCode (Referencing the Google Slide made by Rob Heslin of 340
 * for basic functionality of Swerve Drive).
 * @author Chloe Quinn
 * 
 * @see <a href="https://docs.google.com/presentation/d/1feVl0L5lgIKSZhKCheWgWhkOydIu-ibgdp7oqA0yqAQ/edit?usp=sharing">340's Swerve Programming Slide</a>
 */
public class SwerveDriveSystem extends DriveSystem{

    private static SwerveModule swerveModules[];
    private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;
    public SwerveDriveKinematics driveKinematics;
    public SwerveDriveOdometry driveOdometry;

    /**
     * Creates a SwerveDriveSystem with only one parameter. (Modify DriveConstants and PID Constants before using)
     * @param controller XboxController
     */
    public SwerveDriveSystem(XboxController controller) {
        super(controller, "swerve");
        frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_MOVE_MOTOR, DriveConstants.FRONT_LEFT_ROTATE_MOTOR, DriveConstants.FRONT_LEFT_ROTATE_SENSOR, false);
        rearLeft = new SwerveModule(DriveConstants.REAR_LEFT_MOVE_MOTOR, DriveConstants.REAR_LEFT_ROTATE_MOTOR, DriveConstants.REAR_LEFT_ROTATE_SENSOR, false);
        rearRight = new SwerveModule(DriveConstants.REAR_RIGHT_MOVE_MOTOR, DriveConstants.REAR_RIGHT_ROTATE_MOTOR, DriveConstants.REAR_RIGHT_ROTATE_SENSOR, false);
        frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_MOVE_MOTOR, DriveConstants.FRONT_RIGHT_ROTATE_MOTOR, DriveConstants.FRONT_RIGHT_ROTATE_SENSOR, false);
        
        swerveModules = new SwerveModule[]{
            frontLeft,
            rearLeft,
            rearRight,
            frontRight
        };

        driveKinematics = new SwerveDriveKinematics(
            DriveConstants.FRONTLEFTT_TRANSLATION2D, DriveConstants.BACKLEFT_TRANSLATION2D, 
            DriveConstants.BACKRIGHT_TRANSLATION2D, DriveConstants.FRONTRIGHT_TRANSLATION2D);
        driveOdometry = new SwerveDriveOdometry(driveKinematics, getGyro().getRotation2d(), getSwerveModulePositions());
    }

     

    @Override
    public void periodic() {
        super.periodic();
        for (int i=0; i<4; i++){
          SwerveModule mod = swerveModules[i];
          if(mod.rotateMotor.exporting) {
            mod.rotateMotor.exportPeriodic();
          }
        }
        driveOdometry.update(getGyroRot2d(), getSwerveModulePositions());
    }

    
    @Override
    public void driveRobotCentricMethod(ChassisSpeeds speeds) {
        ChassisSpeeds chassisSpeeds = controllerToHolonomicSpeeds();
        if(speeds != null) chassisSpeeds = speeds;
        driveRobotCentric(chassisSpeeds);
    }

    @Override
    public void driveFieldCentricMethod(ChassisSpeeds speeds) {
        ChassisSpeeds chassisSpeeds = controllerToHolonomicSpeeds();

        if(speeds != null) chassisSpeeds = speeds;
        //Incorporate field relative
        driveRobotCentric(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.getGyro().getRotation2d()));
    }


    @Override
    public void driveAllControlPercent(double input) {
        this.currentAppliedInput = input;
        if(!spinExporting) {
          driveRobotCentricDutyCycle(new ChassisSpeeds(input, 0.0d, 0.0d));
        } else {
          driveRobotCentricDutyCycle(new ChassisSpeeds(0.0d, 0.0d, input));
        }
    }
    

    @Override
    public double getFrontLeftVelocity() {
        return getAllModuleVelocity()[0];
    }

    /**
     * Set the SwerveModuleStates of all while desaturating WheelSpeeds 1.0 being the highest possible value
     * @param states Desired SwerveModuleStates
     */
    public void setModuleStatesDutyCycle(SwerveModuleState[] states) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.0d);
      for (int i = 0; i < states.length; i++) {
          swerveModules[i].setModuleStateDutyCycle(states[i]);
      }
    }

    /**
     * Set the SwerveModuleStates of all while desaturating WheelSpeeds with Max Velocity Constant.
     * @param states Desired SwerveModuleStates
     */
    public void setModuleStatesVelocity(SwerveModuleState[] states) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_VELOCITY);
      for (int i = 0; i < states.length; i++) {
          swerveModules[i].setModuleState(states[i]);
      }
    }

    /**
     * Drive with a RobotCentric heading, expecting ChassisSpeeds to contain doubles that are in velocity (m/s) form.
     * @param chassisSpeeds Desired ChassisSpeeds
     */
    public void driveRobotCentric(ChassisSpeeds chassisSpeeds){
      setModuleStatesVelocity(driveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Drive the robot but expecting ChassisSpeeds to contain doubles that are in ControlPercent (DutyCycle) form.
     * Aka -1.0 through 1.0 with 0.0 being at rest.
     * @param chassisSpeeds Desired ChassisSpeeds
     */ 
    public void driveRobotCentricDutyCycle(ChassisSpeeds chassisSpeeds){
      setModuleStatesDutyCycle(driveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Drive the robot but only change the rotation of each module, keep the current velocity.
     * @param desiredAngle - Desired Angle in Degrees
     */
    public void driveRobotCentricRotOnlyKeepVelo(double desiredAngle){
        SwerveModuleState[] targetStates = getAllModuleStates();
        for(int i = 0 ; i < targetStates.length; i++) {
            targetStates[i].angle = Rotation2d.fromDegrees(desiredAngle);
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.MAX_VELOCITY);
        for (int i = 0; i < targetStates.length; i++) {
            swerveModules[i].setModuleStateRotKeepSpeed(desiredAngle);
        }
    }

    /**
     * Drive while converitng double speeds to RobotCentric ChassisSpeeds
     * @param awaySpeed x speed (double)
     * @param lateralSpeed y speed (double)
     * @param rotSpeed rotate speed (double)
     */
    public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed){
        driveRobotCentric(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotSpeed));
    }

    /**
     * Drive while converting double speeds to FieldReltive ChassisSpeeds 
     * @param awaySpeed x speed (double)
     * @param lateralSpeed y speed (double)
     * @param rotSpeed rotate speed (double)
     */
    public void driveFieldRelative(double awaySpeed, double lateralSpeed, double rotSpeed){
        driveRobotCentric(ChassisSpeeds.fromFieldRelativeSpeeds(awaySpeed, lateralSpeed, rotSpeed, getGyroRot2d()));
    }

    /**
     * Drive by a specific module number
     * @param moduleNumber int
     * @param moveSpeed drive speed (double)
     * @param rotatePos rotate angle (double)
     * @author Rob
     */
    public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos){
        if (moduleNumber > 3 || moduleNumber < 0){
            System.out.println("Module " + moduleNumber + " is out of bounds.");
            return;
        }else if(rotatePos < -Math.PI || rotatePos > Math.PI){
            System.out.println("Input angle out of range.");
            return;
        }

        SwerveModuleState oneSwerveState = new SwerveModuleState(moveSpeed, new Rotation2d(rotatePos));
        swerveModules[moduleNumber].setModuleState(oneSwerveState);
    }

    /**
     * Command that enables PIDExporting on the Front Left module.
     * @return WPILIB Command
     */
    public Command enableOneModuleRotExport() {
        return Commands.runOnce(() -> swerveModules[0].rotateMotor.enablePIDExport());
    }

    /**
     * Set all modules motors to 0.- percent input
     */
    public void stopAllModules(){
        for (int i=0; i<4; i++){
            swerveModules[i].stopAll();
        }
    }

    /**
     * Get the Odometry's current positon
     * @return Robot position as Pose2d
     */
    public Pose2d getCurPose2d(){
        return driveOdometry.getPoseMeters();
    }

    /**
     * Reset the odometry's current position to the odometry's current position??? (I don't really know, I will ask Rob next chance I get)
     */
    public void resetPose() {
        driveOdometry.resetPosition(getGyroRot2d(), getSwerveModulePositions(), getCurPose2d());
    }

    /**
     * Reset the odometry's current positon to a specific Pose2d
     * @param pose Pose2d
     */
    public void setCurPose2d(Pose2d pose) {
        driveOdometry.resetPosition(getGyroRot2d(), getSwerveModulePositions(), pose);
    }


    /**
     * Get distance traveled (meters) of each module.
     * @return Array of doubles double[].
     */
    public double[] getAllModuleDistance(){
        double[] moduleDistances = new double[4];
        for(int i=0; i<4; i++){
            moduleDistances[i]=swerveModules[i].getDriveDistance();
        }
        return moduleDistances;
    }

    /**
     * Get applied velocity (m/s) of each respective module.
     * @return Array of doubles double[].
     */
    public double[] getAllModuleVelocity(){
        double[] moduleVelocities = new double[4];
        for(int i=0; i<4; i++){
            moduleVelocities[i]=swerveModules[i].getDriveVelocity();
        }
        return moduleVelocities;
    }

    /**
     * Get the applied PercentOutput (-1.0 to 1.0, with 0.0 being at rest) of each respective module.
     * @return Array of doubles double[].
     */
    public double[] getAllModulePercentOutput(){
        double[] modules = new double[4];
        for(int i=0; i<4; i++){
            modules[i]=swerveModules[i].getDriveInput();
        }
        return modules;
    }

    /**
     * Get the states of all Swerve Modules as an array
     * @return Array of SwerveModuleStates.
     */
    public SwerveModuleState[] getAllModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i=0; i<4; i++){
            states[i]=swerveModules[i].getModuleState();
        }
        return states;
    }

    /**
     * Get positions of all modules as an array.
     * @return Array of SwerveModulePositions
     */
    public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = swerveModules[i].getModulePosition();
        }
        return modulePositions;
    }

    /**
     * Set the same PIDF (Porportional, Integral, Derivative, FeedForward)
     * @param P - Porportional gain
     * @param I - Integral gain (NOT RECOMMENDED)
     * @param D - Derivative gain
     * @param F - Feedforward gain (In Rob we trust)
     */
    public void setDrivePIDF(double P, double I, double D, double F){
        for (int i=0; i<4; i++){
            swerveModules[i].setDriveMotorPIDF(P, I, D, F);
        }
    }


    /**
     * Zero all CANCoders.
     */
    public void zeroAllModulePosSensors(){
        for (int i=0; i<4; i++){
            swerveModules[i].zeroAbsPositionSensor();
        }
    }

    /**
     * Change the zero of a respective CANCoder.
     * @param modNumber Swerve module number
     */
    public void zeroModulePosSensor(int modNumber){
        swerveModules[modNumber].zeroAbsPositionSensor();
    }
}
