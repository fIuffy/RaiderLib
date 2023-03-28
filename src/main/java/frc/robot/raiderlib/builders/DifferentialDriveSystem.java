package frc.robot.raiderlib.builders;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.raiderlib.drive.DriveConstants;
import frc.robot.raiderlib.drive.DriveSystem;
import frc.robot.raiderlib.motor.struct.MotorControllerSimple;
import frc.robot.raiderlib.motor.struct.MotorControllerSimple.CommonControllers;

public class DifferentialDriveSystem extends DriveSystem{

    private final DifferentialDriveKinematics driveKinematics;
    private final DifferentialDriveOdometry driveOdometry;

    public final MotorControllerSimple leftFront = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.LEFT_FRONT, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);
    public final MotorControllerSimple leftRear = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.LEFT_REAR, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);

    public final MotorControllerSimple rightFront = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.RIGHT_FRONT, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);
    public final MotorControllerSimple rightRear = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.RIGHT_REAR, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);

    /**
     * Creates a DifferentialDriveSystem with only one parameter. (Modify DriveConstants and PID Constants before using)
     * @param controller XboxController
     */
    public DifferentialDriveSystem(XboxController controller) {
        super(controller, "differential");
        this.driveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.DIFFERENTIAL_TRACK_WIDTH));
        this.driveOdometry = new DifferentialDriveOdometry(this.getGyro().getRotation2d(), 0.0d, 0.0d);
        this.leftFront.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        this.leftRear.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        this.rightFront.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        this.rightRear.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        
        this.exportStepTime = 1.0d;
    }

    @Override
    public void periodic() {
        super.periodic();
        this.setCurrentPose(driveOdometry.update(this.getGyro().getRotation2d(), this.leftFront.getMotor().getMotorPositionConverted(), this.rightFront.getMotor().getMotorPositionConverted()));
    }

    @Override
    public void driveRobotCentricMethod(ChassisSpeeds speeds) {
        ChassisSpeeds chassisSpeeds = controllerToDifferentialSpeeds();
        if(speeds != null) chassisSpeeds = speeds;

        DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
        outputWheelSpeeds(wheelSpeeds);
    }

    /**
     * DifferentialDriveWheelSpeeds consumer used for driving ChassisSpeeds but assuming the inputs are in Velocity form
     * @param speeds - Wheel speeds
     */
    public void outputWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftControlPercent = speeds.leftMetersPerSecond;
        double rightControlPercent = speeds.rightMetersPerSecond;

        leftFront.getMotor().setMotorVelocity(leftControlPercent);
        leftRear.getMotor().setMotorVelocity(leftControlPercent);

         /** Incorporate Rear Motors (Delete if not used) */
        rightFront.getMotor().setMotorVelocity(rightControlPercent);
        rightRear.getMotor().setMotorVelocity(rightControlPercent);
    }

    /**
     * DifferentialDriveWheelSpeeds consumer used for driving ChassisSpeeds but assuming the inputs are in ControlPercent form
     * @param speeds - Wheel speeds
     */
    public void outputWheelSpeedsDutyCycle(DifferentialDriveWheelSpeeds speeds) {
        double leftControlPercent = speeds.leftMetersPerSecond;
        double rightControlPercent = speeds.rightMetersPerSecond;

        leftFront.getMotor().setMotorControlPercent(leftControlPercent);
        leftRear.getMotor().setMotorControlPercent(leftControlPercent);

         /** Incorporate Rear Motors (Delete if not used) */
        rightFront.getMotor().setMotorControlPercent(rightControlPercent);
        rightRear.getMotor().setMotorControlPercent(rightControlPercent);
    }

    @Override
    public void driveAllControlPercent(double input) {
        ChassisSpeeds speeds;
        this.currentAppliedInput = input;
        if(!spinExporting) {
            speeds = new ChassisSpeeds(input, 0.0d, 0.0d);
        } else {
            speeds = new ChassisSpeeds(0.0d, 0.0d, input);
        }
        outputWheelSpeedsDutyCycle(driveKinematics.toWheelSpeeds(speeds));
    }

    @Override
    public double getFrontLeftVelocity() {
        return leftFront.getMotor().getMotorVelocityConverted();
    }
    
    /**
     * Double BiConsumer used by PathPlanner for setting motor speeds
     * @param leftVolts - Input of left motors, in volts
     * @param rightVolts - Input of right motors, in volts
     */
    public void outputVolts(double leftVolts, double rightVolts) {
        double leftInput = 12.0d / leftVolts;
        double rightInput = 12.0d / rightVolts;

        leftFront.getMotor().setMotorControlPercent(leftInput);
        leftRear.getMotor().setMotorControlPercent(leftInput);
        /** Incorporate Rear Motors (Delete if not used) */
        rightFront.getMotor().setMotorControlPercent(rightInput);
        rightRear.getMotor().setMotorControlPercent(rightInput);
    }

    /**
     * 
     * @return DifferentialDriveOdometry main kinematics object
     */
    public DifferentialDriveKinematics getKinematics() {
        return this.driveKinematics;
    }

    /**
     * 
     * @return DifferentialDriveOdometry main odometry object
     */
    public DifferentialDriveOdometry getOdometry() {
        return this.driveOdometry;
    }
}