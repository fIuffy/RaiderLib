package frc.robot.raiderlib.builders;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.raiderlib.drive.DriveConstants;
import frc.robot.raiderlib.drive.DriveSystem;
import frc.robot.raiderlib.motor.struct.MotorControllerSimple;
import frc.robot.raiderlib.motor.struct.MotorControllerSimple.CommonControllers;

public class MecanumDriveSystem extends DriveSystem{

    private final MecanumDriveKinematics driveKinematics;
    private final MecanumDriveOdometry driveOdometry;

    public final MotorControllerSimple leftFront = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.LEFT_FRONT, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);
    public final MotorControllerSimple leftRear = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.LEFT_REAR, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);

    public final MotorControllerSimple rightFront = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.RIGHT_FRONT, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);
    public final MotorControllerSimple rightRear = new MotorControllerSimple(CommonControllers.TALON_FX, DriveConstants.RIGHT_REAR, true, null, DriveConstants.MIN_DRIVE_DUTYCYCLE,
                                                                            true, DriveConstants.MAX_CONTROLPERCENT, false);

    /**
     * Creates a MecanumDriveSystem with only one parameter. (Modify DriveConstants and PID Constants before using)
     * @param controller XboxController
     */
    public MecanumDriveSystem(XboxController controller) {
        super(controller, "mecanum");
        this.driveKinematics = new MecanumDriveKinematics(DriveConstants.FRONTLEFTT_TRANSLATION2D, DriveConstants.FRONTRIGHT_TRANSLATION2D, DriveConstants.BACKLEFT_TRANSLATION2D, DriveConstants.BACKRIGHT_TRANSLATION2D);
        this.driveOdometry = new MecanumDriveOdometry(driveKinematics,
            this.getGyro().getRotation2d(),
            new MecanumDriveWheelPositions(
                leftFront.getMotor().getMotorPositionConverted(), rightFront.getMotor().getMotorPositionConverted(),
                leftRear.getMotor().getMotorPositionConverted(), rightRear.getMotor().getMotorPositionConverted()),
            new Pose2d(0.0d, 0.0d, new Rotation2d()));
        this.leftFront.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        this.leftRear.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        this.rightFront.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        this.rightRear.getMotor().setEncoderConversionFactor(DriveConstants.ENC_TO_METERS_FACTOR);
        this.exportStepTime = 1.0d;
    }

    @Override
    public void periodic() {
        super.periodic();
        MecanumDriveWheelPositions wheelPoses = new MecanumDriveWheelPositions(
                leftFront.getMotor().getMotorPositionConverted(), rightFront.getMotor().getMotorPositionConverted(),
                leftRear.getMotor().getMotorPositionConverted(), rightRear.getMotor().getMotorPositionConverted());
        this.setCurrentPose(driveOdometry.update(this.getGyro().getRotation2d(), wheelPoses));
    }

    @Override
    public void driveRobotCentricMethod(ChassisSpeeds speeds) {
        ChassisSpeeds chassisSpeeds = controllerToHolonomicSpeeds();
        if(speeds != null) chassisSpeeds = speeds;
        outputWheelSpeeds(driveKinematics.toWheelSpeeds(chassisSpeeds));
    }

    @Override
    public void driveFieldCentricMethod(ChassisSpeeds speeds) {
        ChassisSpeeds chassisSpeeds = controllerToHolonomicSpeeds();
        if(speeds != null) chassisSpeeds = speeds;

        //Incorporate field relative
        outputWheelSpeeds(driveKinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, this.getGyro().getRotation2d())));
    }

    /**
     * MecanumDriveWheelSpeeds consumer used for driving ChassisSpeeds but assuming the inputs are in Velocity form
     * @param speeds - Wheel speeds
     */
    public void outputWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
        double leftFrontVelocity = speeds.frontLeftMetersPerSecond;
        double leftRearVelocity = speeds.rearLeftMetersPerSecond;
        
        double rightFrontVelocity = speeds.frontRightMetersPerSecond;
        double rightRearVelocity = speeds.rearRightMetersPerSecond;

        leftFront.getMotor().setMotorVelocity(leftFrontVelocity);
        leftRear.getMotor().setMotorVelocity(leftRearVelocity);

        rightFront.getMotor().setMotorVelocity(rightFrontVelocity);
        rightRear.getMotor().setMotorVelocity(rightRearVelocity);
    }

    /**
     * MecanumDriveWheelSpeeds consumer used for driving ChassisSpeeds but assuming the inputs are in ControlPercent form
     * @param speeds - Wheel speeds
     */
    public void outputWheelSpeedsDutyCycle(MecanumDriveWheelSpeeds speeds) {
        double leftFrontVelocity = speeds.frontLeftMetersPerSecond;
        double leftRearVelocity = speeds.rearLeftMetersPerSecond;
        
        double rightFrontVelocity = speeds.frontRightMetersPerSecond;
        double rightRearVelocity = speeds.rearRightMetersPerSecond;

        leftFront.getMotor().setMotorControlPercent(leftFrontVelocity);
        leftRear.getMotor().setMotorControlPercent(leftRearVelocity);

        rightFront.getMotor().setMotorControlPercent(rightFrontVelocity);
        rightRear.getMotor().setMotorControlPercent(rightRearVelocity);
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
     * 
     * @return MecanumDriveKinematics main kinematics object
     */
    public MecanumDriveKinematics getKinematics() {
        return this.driveKinematics;
    }

    /**
     * 
     * @return MecanumDriveOdometry main odometry object
     */
    public MecanumDriveOdometry getOdometry() {
        return this.driveOdometry;
    }
    
}
