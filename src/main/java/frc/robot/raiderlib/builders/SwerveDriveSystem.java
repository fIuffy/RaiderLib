package frc.robot.raiderlib.builders;

import java.math.BigDecimal;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.raiderlib.drive.struct.DriveConstants;
import frc.robot.raiderlib.drive.struct.DriveSystem;
import frc.robot.raiderlib.drive.swerve.SwerveModule;

public class SwerveDriveSystem extends DriveSystem{

    private static SwerveModule swerveModules[];
    private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;
    public SwerveDriveKinematics driveKinematics;
    public SwerveDriveOdometry driveOdometry;
    private PIDController robotSpinController;

    public boolean spinExporting;
    public double currentInput;

    public SwerveDriveSystem(XboxController controller, String exportName) {
        super(controller, exportName);
        // Constructs the swerve modules 
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
        
        //construct the odometry class.
        driveOdometry = new SwerveDriveOdometry(driveKinematics, getGyro().getRotation2d(), getSwerveModulePositions());

        //construct the wpilib PIDcontroller for rotation.
        robotSpinController = new PIDController(DriveConstants.ROBOT_ROT_P, DriveConstants.ROBOT_ROT_I, DriveConstants.ROBOT_ROT_D);
        robotSpinController.setTolerance(DriveConstants.ROBOT_ROT_TOLERANCE);
        spinExporting = false;
        currentInput = 0.0d;
    }

    /**
     * This enumeration clarifies the numbering of the swerve module for new users.
     * frontLeft  | 0
     * rearLeft   | 1
     * rearRight  | 2
     * frontRight | 3
     */
    public enum SwerveModNum{
        frontLeft(0),
        rearLeft(1),
        rearRight(2),
        frontRight(3);
        public int num;
        private SwerveModNum(int number){
            num = number;
        }
        public int getNumber() {
            return num;
        }
    }

    public void enableSpinExporting() {
      this.spinExporting = true;
    }

    public void disableSpinExporting() {
      this.spinExporting = false;
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
        driveOdometry.update(getGyroRotation2d(), getSwerveModulePositions());
    }

    
    @Override
    public void driveRobotCentricMethod(ChassisSpeeds speeds) {
        double moveLateralControlPercent = getDriverAxis(Axis.kLeftY, getController());
        double moveStrafeControlPercent = getDriverAxis(Axis.kLeftX, getController());
        double rotateControlPercent = getDriverAxis(Axis.kRightX, getController());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(moveLateralControlPercent*DriveConstants.MAX_VELOCITY, moveStrafeControlPercent*DriveConstants.MAX_VELOCITY, rotateControlPercent*DriveConstants.MAX_ANGULAR_VELOCITY);

        if(speeds != null) chassisSpeeds = speeds;
        driveRobotCentric(chassisSpeeds);
    }

    public double getDriverAxis(Axis axis) {
      return (getController().getRawAxis(axis.value) < -.1 || getController().getRawAxis(axis.value) > .1)
          ? getController().getRawAxis(axis.value)
          : 0.0;
    }

    @Override
    public void driveFieldCentricMethod(ChassisSpeeds speeds) {
        double moveLateralControlPercent = getDriverAxis(Axis.kLeftY, getController());
        double moveStrafeControlPercent = getDriverAxis(Axis.kLeftX, getController());
        double rotateControlPercent = getDriverAxis(Axis.kRightX, getController());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(moveLateralControlPercent*DriveConstants.MAX_VELOCITY, moveStrafeControlPercent*DriveConstants.MAX_VELOCITY, rotateControlPercent*DriveConstants.MAX_ANGULAR_VELOCITY);

        if(speeds != null) chassisSpeeds = speeds;
        //Incorporate field relative
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getGyro().getRotation2d());
        driveRobotCentric(chassisSpeeds);
    }


    public void driveAllControlPercent(double input) {
        this.currentInput = input;
        if(!spinExporting) {
          driveRobotCentricDutyCycle(new ChassisSpeeds(input, 0.0d, 0.0d));
        } else {
          driveRobotCentricDutyCycle(new ChassisSpeeds(0.0d, 0.0d, input));
        }
    }

    @Override
    public void exportPeriodic() {
        long difference = System.currentTimeMillis() - startTime;
        BigDecimal bd = BigDecimal.valueOf(difference).movePointLeft(3);
        double diffSeconds = bd.doubleValue();
        switch((int)Math.floor(diffSeconds/this.exportStepTime)) {
            case 0: this.driveAllControlPercent(0.0d); resetGyro(); break;
            case 1: this.driveAllControlPercent(DriveConstants.MIN_DRIVE_DUTYCYCLE); break;
            case 2: this.driveAllControlPercent(0.0d); break;
            case 3: this.driveAllControlPercent(-DriveConstants.MIN_DRIVE_DUTYCYCLE); break;
            case 4: this.driveAllControlPercent(0.0d); break;
            case 5: this.driveAllControlPercent(DriveConstants.MIN_DRIVE_DUTYCYCLE*2); break;
            case 6: this.driveAllControlPercent(0.0d); break;
            case 7: this.driveAllControlPercent(-DriveConstants.MIN_DRIVE_DUTYCYCLE*2); break;
            case 8: this.driveAllControlPercent(0.0d); break;
            default: this.disablePIDExport(); return;
        }
        /**
         * Actually append to the FileWriter
         * We use append so that we can just simply add on without needed to write to the file through one object.
         */
        try {
            String writeString = diffSeconds+","+this.currentInput+","+(getAllModuleVelocity()[0]) + "\n";
            double rotationalVeloInRad = getRotationalVelocity() * (Math.PI/180d);
            if(spinExporting) writeString = diffSeconds+","+rotationalVeloInRad+","+(getGyroInRad()) + "\n";
            writer.append(writeString);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

  public void setChassisSpeedsDutyCycle(ChassisSpeeds speeds) {
    SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.0d);
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i]);
    }
  }

  public void setChassisSpeedsVelocity(ChassisSpeeds speeds) {
    SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_VELOCITY);
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i]);
    }
  }

  public void setModuleStatesDutyCycle(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.0d);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i]);
    }
  }

  public void setModuleStatesVelocity(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_VELOCITY);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i]);
    }
  }

  /* =================== Module Drive Methods =================== */

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param chassisSpeeds an object  
   */
  // Velocity control driving
  public void driveRobotCentric(ChassisSpeeds chassisSpeeds){
    //instantiate an array of SwerveModuleStates, set equal to the output of toSwerveModuleStates() 
    SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(chassisSpeeds);
    //use SwerveDriveKinematic.desaturateWheelSpeeds(), max speed is 1 if percentOutput, MaxVelovcity if velocity mode
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.MAX_VELOCITY);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < targetStates.length; i++) {
        swerveModules[i].setModuleState(targetStates[i]);
    }
  }

  // DutyCycle control driving
  public void driveRobotCentricDutyCycle(ChassisSpeeds chassisSpeeds){
    //instantiate an array of SwerveModuleStates, set equal to the output of toSwerveModuleStates() 
    SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(chassisSpeeds);
    //use SwerveDriveKinematic.desaturateWheelSpeeds(), max speed is 1 if percentOutput, MaxVelovcity if velocity mode
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 1.0d);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < targetStates.length; i++) {
        swerveModules[i].setModuleStateDutyCycle(targetStates[i]);
    }
  }

  public void driveRobotCentricRotOnlyKeepVelo(ChassisSpeeds chassisSpeeds, double desiredAngle){
    //instantiate an array of SwerveModuleStates, set equal to the output of toSwerveModuleStates() 
    SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(chassisSpeeds);
    //use SwerveDriveKinematic.desaturateWheelSpeeds(), max speed is 1 if percentOutput, MaxVelovcity if velocity mode
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.MAX_VELOCITY);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < targetStates.length; i++) {
        swerveModules[i].setModuleStateRotKeepSpeed(desiredAngle);
    }
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param forwardSpeed the movement forward and backward
   * @param strafeSpeed the movement side to side
   * @param rotSpeed the speed of rotation
   */
  public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed){
    //convert forwardSpeed, strafeSpeed and rotSpeed to a chassisSpeeds object, pass to driveRobotCentric
    driveRobotCentric(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotSpeed));
  }

  /**
   * Drive the robot so that all directions are independent of the robots orientation (rotation)
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not moving in that direction
   * 
   * @param awaySpeed from field relative, aka a fix direction,
   *                  away from or toward the driver, a speed
   *                  valued between -1.0 and 1.0, where 1.0
   *                  is to away from the driver 
   * @param lateralSpeed from field relative, aka a fix direction
   *                     regardless of robot rotation, a speed
   *                     valued between -1.0 and 1.0, where 1.0
   *                     is to the left 
   * @param rotSpeed rotational speed of the robot
   *                 -1.0 to 1.0 where 0.0 is not rotating
   */
  public void driveFieldRelative(double awaySpeed, double lateralSpeed, double rotSpeed){
    driveRobotCentricMethod(ChassisSpeeds.fromFieldRelativeSpeeds(awaySpeed, lateralSpeed, rotSpeed, getGyroRotation2d()));
  }

    /**
     * This function is meant to drive one module at a time for testing purposes.
     * @param moduleNumber which of the four modules(0-3) we are using
     * @param moveSpeed move speed -1.0 to 1.0, where 0.0 is stopped
     * @param rotatePos a position between -PI and PI where we want the module to be
     */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos){
    //test that moduleNumber is between 0-3, return if not(return;)
    if (moduleNumber > 3 && moduleNumber < 0){
      System.out.println("Module " + moduleNumber + " is out of bounds.");
      return;
    }else if(rotatePos < -Math.PI || rotatePos > Math.PI){
      System.out.println("Input angle out of range.");
      return;
    }

    SwerveModuleState oneSwerveState = new SwerveModuleState(moveSpeed, new Rotation2d(rotatePos));
    //code to drive one module in a testing form
    swerveModules[moduleNumber].setModuleState(oneSwerveState);

  }

  public Command enableOneModuleRotExport() {
    return Commands.runOnce(() -> swerveModules[0].rotateMotor.enablePIDExport());
  }

  /**
   * Stops all module motion, then lets all the modules spin freely.
   */
  public void stopAllModules(){
    //run a for loop to call each mmodule
    for (int i=0; i<4; i++){
      //use the stopAll method, which stops both the drive and rotation motor.
      swerveModules[i].stopAll();
    }
  }

  /* =================== Robot Pose Methods =================== */
  
  /**
   * Pull the current Position from the odometry 
   * class, this should be in meters.
   * 
   * @return a Pose2d representing the current position
   */
  public Pose2d getCurPose2d(){
    return driveOdometry.getPoseMeters();
  }

  public void resetPose() {
    driveOdometry.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), getCurPose2d());
  }


  /**
   * Sets current position in the odometry class
   * 
   * @param pose new current position
   */
  public void setCurPose2d(Pose2d pose) {
    driveOdometry.resetPosition( getGyroRotation2d(), getSwerveModulePositions(), pose);
  }

  /* =================== Gyro/IMU Methods =================== */
  
  /**
   * A function that allows the user to reset the gyro, this 
   * makes the current orientation of the robot 0 degrees on 
   * the gyro.
   */
  public void resetGyro(){
    //Resets the gyro(zero it)
    getGyro().reset();
  }

  /**
   * A function that allows the user to set the gyro to a 
   * specific angle. This will make the current orientation 
   * of the robot the input value. This must be in degrees 
   * for gyro.
   * @param newCurrentAngle value the gyro should now read in degrees.
   */
  public void setGyro(double newCurrentAngle){
    getGyro().reset();
    getGyro().setAngleAdjustment(newCurrentAngle);
  }

  
  /**
   * This calls the drive Gyro and returns the Rotation2d object.
   * This object contains angle in radians, as well as the sin 
   * and cos of that angle. This is an object that represents the
   * rotation of the robot.
   * @return a Rotation2d object
   */
  public Rotation2d getGyroRotation2d(){
    //return a newly constructed Rotation2d object, it takes the angle in radians as a constructor argument
    return Rotation2d.fromDegrees(getGyroInDeg());
    //note that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
    return Math.toRadians(getGyroInDeg());// Pull the gyro in degrees, convert and return in radians
    //note that counterclockwise rotation is positive
  }


  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return getGyro().getAngle()*-1;//Pull gyro in degrees
    //note counterclockwise rotation is positive
  }

  /**
   * Returns the speed of rotation of the robot, 
   * counterclockwise is positive.
   * @return degrees per second
   */
  public double getRotationalVelocity(){
    return getGyro().getRate()*-1;
  }

  /* =================== Pull From All Module Methods =================== */
  
  /**
   * Returns all values from the module's absolute 
   * encoders, and returns them in an array of 
   * doubles, as degrees, in module order.
   * 
   * @return array of doubles, in degrees
   */
  public double[] getAllAbsModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getAbsPosInDeg();
    }
    return moduleAngles;
  }

  public void resetContinuousRotPos(int modNumber) {
    swerveModules[modNumber].resetRotationSensorPosition();
  }
  

  /**
   * Returns all values from the module's absolute 
   * encoders, and returns them in an array of 
   * doubles, as RADIANS, in module order.
   * 
   * @return array of doubles, in radians
   */
  public double[] getAllAbsModuleAnglesRad(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getPosInRad();
    }
    return moduleAngles;
  }

  /**
   * Returns all values from the rotational motor's 
   * reletive encoders in an array of doubles. This 
   * array is in order of module number.
   * 
   * @return array of doubles, representing tick count.
   */
  public double[] getAllModuleRelEnc(){
    double[] moduleRelEnc = new double[4];
    for(int i=0; i<4; i++){
      moduleRelEnc[i]=swerveModules[i].getRelEncCount();
    }
    return moduleRelEnc;
  }

  /**
   * Returns the collective distance as seen by the 
   * drive motor's encoder, for each module.
   * 
   * @return an array of doubles in meters
   */
  public double[] getAllModuleDistance(){
    double[] moduleDistances = new double[4];
    for(int i=0; i<4; i++){
      moduleDistances[i]=swerveModules[i].getDriveDistance();
    }
    return moduleDistances;
  }

  /**
   *  Gets all the drive velocities.
   * 
   * @return An array of velocities.
   */
  public double[] getAllModuleVelocity(){
    double[] moduleVelocities = new double[4];
    for(int i=0; i<4; i++){
      moduleVelocities[i]=swerveModules[i].getDriveVelocity();
    }
    return moduleVelocities;
  }

  public double[] getAllModulePercentOutput(){
    double[] modules = new double[4];
    for(int i=0; i<4; i++){
        modules[i]=swerveModules[i].getDriveInput();
    }
    return modules;
  }

  /**
   * Gets all the SwerveModulePosition 
   * of all modules.
   * 
   * @return An array of SwerveModulePositions.
   */
  public SwerveModulePosition[] getSwerveModulePositions(){
    //instatiate and construct a 4 large SwerveModuleState array
    SwerveModulePosition[] modulePositions =  new SwerveModulePosition[4];
    //get the current SwerveModuleStates from all modules in array
    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = swerveModules[i].getModulePosition();
    }

    return modulePositions;
  }
  
  /**  
   * method to configure all modules DriveMotor PIDF
   * these are the PIDF on the TalonFX. This is for
   * testing
   */
  public void setDrivePIDF(double P, double I, double D, double F){
    for (int i=0; i<4; i++){
      swerveModules[i].setDriveMotorPIDF(P, I, D, F);
    }
  }

  
  /**
   * Method for taking the current position of all modules,
   * and making that position the absolute zero of each 
   * modules position respectively.
   */
  public void zeroAllModulePosSensors(){
    //a for loop so cycle through all modules
    for (int i=0; i<4; i++){
      //call the zero position method
      swerveModules[i].zeroAbsPositionSensor();
    }
  }

  public void zeroModulePosSensor(int modNumber){
      swerveModules[modNumber].zeroAbsPositionSensor();
  }

  /**
   * 
   * @param target an angle in radians
   * @return a value to give the rotational input, -1.0 to 1.0
   */
  public double getRobotRotationPIDOut(double target){
    double currentGyroPos = getGyroInRad();
    double output = robotSpinController.calculate(currentGyroPos, target);
    if(robotSpinController.atSetpoint()){
      return 0.0;
    } else {
      if (Math.abs(output) < DriveConstants.MIN_ROT_OUTPUT){
        return DriveConstants.MIN_ROT_OUTPUT*Math.signum(output);
      }else {
        return output;
      }
    }
  }
}
