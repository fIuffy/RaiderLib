package me.chloe.raiderlib.builders.drivesystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import me.chloe.raiderlib.RaiderLib;
import me.chloe.raiderlib.builders.MotorControllerSimple;
import me.chloe.raiderlib.drive.DriveConstants;
import me.chloe.raiderlib.drive.DriveSystem;
import me.chloe.raiderlib.motor.CommonControllers;

/**
 * Basic drive train with four motors, two on the left, two on the right.
 */
public class DifferentialDriveSystem extends DriveSystem{

    private final DifferentialDriveKinematics driveKinematics;
    private final DifferentialDriveOdometry driveOdometry;
    private final RamseteController ramseteController;

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
        this.ramseteController = new RamseteController();
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
    
    @Override
    public void zeroDriveSensors() {
        leftFront.getMotor().resetMotorPosition();
        leftRear.getMotor().resetMotorPosition();
        rightFront.getMotor().resetMotorPosition();
        rightRear.getMotor().resetMotorPosition();
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
    public void resetOdometry(Pose2d pose2d) {
        this.driveOdometry.resetPosition(getGyroRot2d(), 0.0d, 0.0d, pose2d);
    }

    @Override
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            if(isFirstPath){
                this.resetOdometry(traj.getInitialPose());
            }
            }),
            new PPRamseteCommand(
                traj,
                this::getCurrentPose,
                this.ramseteController,
                this.driveKinematics,
                this::outputMetersPerSecond,
                true,
                RaiderLib.INSTANCE
            )
        );
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
    public double getAvgDriveVelociy() {
        return (leftFront.getMotor().getMotorVelocityConverted() + leftRear.getMotor().getMotorVelocityConverted()
        + rightFront.getMotor().getMotorVelocityConverted() + rightRear.getMotor().getMotorVelocity()) / 4;
    }
    
    /**
     * Set the motors based on velocity.
     * @param leftVelocity - Input of left motors, in velocity
     * @param rightVelocity - Input of right motors, in velocity
     */
    public void outputMetersPerSecond(double leftVelocity, double rightVelocity) {
        leftFront.getMotor().setMotorVelocity(leftVelocity);
        leftRear.getMotor().setMotorVelocity(leftVelocity);

         /** Incorporate Rear Motors (Delete if not used) */
        rightFront.getMotor().setMotorVelocity(rightVelocity);
        rightRear.getMotor().setMotorVelocity(rightVelocity);
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