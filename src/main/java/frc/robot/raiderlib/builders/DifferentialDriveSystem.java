package frc.robot.raiderlib.builders;

import java.math.BigDecimal;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import frc.robot.raiderlib.drive.struct.DriveConstants;
import frc.robot.raiderlib.drive.struct.DriveSystem;
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

    public DifferentialDriveSystem(XboxController controller, String exportName) {
        super(controller, exportName);
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
        double moveControlPercent = getDriverAxis(Axis.kLeftY, getController());
        double rotateControlPercent = getDriverAxis(Axis.kRightX, getController());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(moveControlPercent*DriveConstants.MAX_VELOCITY, 0, rotateControlPercent*DriveConstants.MAX_ANGULAR_VELOCITY);

        if(speeds != null) chassisSpeeds = speeds;

        DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;

        leftFront.getMotor().setMotorVelocity(leftVelocity);
        leftRear.getMotor().setMotorVelocity(leftVelocity);
        /** Incorporate Rear Motors (Delete if not used) */
        rightFront.getMotor().setMotorVelocity(rightVelocity);
        rightRear.getMotor().setMotorVelocity(rightVelocity);
    }

    public void driveAllControlPercent(double input) {
        leftFront.getMotor().setMotorControlPercent(input);
        leftRear.getMotor().setMotorControlPercent(input);
        /** Incorporate Rear Motors (Delete if not used) */
        rightFront.getMotor().setMotorControlPercent(input);
        rightRear.getMotor().setMotorControlPercent(input);
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

    public DifferentialDriveKinematics getKinematics() {
        return this.driveKinematics;
    }

    public DifferentialDriveOdometry getOdometry() {
        return this.driveOdometry;
    }
    
}
