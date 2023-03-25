package frc.robot.raiderlib.builders;

import java.math.BigDecimal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import frc.robot.raiderlib.drive.struct.DriveConstants;
import frc.robot.raiderlib.drive.struct.DriveSystem;
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

    public MecanumDriveSystem(XboxController controller, String exportName) {
        super(controller, exportName);
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
        double moveLateralControlPercent = getDriverAxis(Axis.kLeftY, getController());
        double moveStrafeControlPercent = getDriverAxis(Axis.kLeftX, getController());
        double rotateControlPercent = getDriverAxis(Axis.kRightX, getController());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(moveLateralControlPercent*DriveConstants.MAX_VELOCITY, moveStrafeControlPercent*DriveConstants.MAX_VELOCITY, rotateControlPercent*DriveConstants.MAX_ANGULAR_VELOCITY);

        if(speeds != null) chassisSpeeds = speeds;

        MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
        outputWheelSpeeds(wheelSpeeds);
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

        MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
        outputWheelSpeeds(wheelSpeeds);
    }

    /**
     * MecanumDriveWheelSpeeds consumer needed for PathPlanner following
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


    public void driveAllControlPercent(double input) {
        leftFront.getMotor().setMotorControlPercent(input);
        leftRear.getMotor().setMotorControlPercent(input);

        rightFront.getMotor().setMotorControlPercent(input);
        rightRear.getMotor().setMotorControlPercent(input);
    }

    @Override
    public void exportPeriodic() {
        long difference = System.currentTimeMillis() - startTime;
        BigDecimal bd = BigDecimal.valueOf(difference).movePointLeft(3);
        double diffSeconds = bd.doubleValue();
        switch((int)Math.floor(diffSeconds/this.exportStepTime)) {
            case 0: this.driveAllControlPercent(0.0d); break;
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
            writer.append(diffSeconds+","+leftFront.getMotor().getMotorOutputPercent()+","+(leftFront.getMotor().getMotorVelocityConverted()) + "\n");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public MecanumDriveKinematics getKinematics() {
        return this.driveKinematics;
    }

    public MecanumDriveOdometry getOdometry() {
        return this.driveOdometry;
    }
    
}
