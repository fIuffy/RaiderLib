package me.chloe.raiderlib.drive;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * Suggested to be changed before performing use on any drive system.
 */
public class DriveConstants {
    /**
     * Four Motor
     */
    public static final int LEFT_REAR = -1;
    public static final int LEFT_FRONT = -1;

    public static final int RIGHT_REAR = -1;
    public static final int RIGHT_FRONT = -1;

    /**
     * Swerve 
     */
    /**
     * Drive Module 0
     */
    public static final int FRONT_LEFT_MOVE_MOTOR = 1;
    public static final int FRONT_LEFT_ROTATE_MOTOR = 2;
    public static final int FRONT_LEFT_ROTATE_SENSOR = 15;

    /**
     * Drive Module 1
     */
    public static final int REAR_LEFT_MOVE_MOTOR = 3;
    public static final int REAR_LEFT_ROTATE_MOTOR = 4;
    public static final int REAR_LEFT_ROTATE_SENSOR = 12;

    /**
     * Drive Module 2
     */
    public static final int REAR_RIGHT_MOVE_MOTOR = 5;
    public static final int REAR_RIGHT_ROTATE_MOTOR = 6;
    public static final int REAR_RIGHT_ROTATE_SENSOR = 13;
    
    /**
     * Drive Module 3
     */
    public static final int FRONT_RIGHT_MOVE_MOTOR = 7;
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 8;
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 14;
    
    /**
     * Translation2d (Location) of Front Left motor (x, y). Measured in meters.
     */
    public static final Translation2d FRONTLEFTT_TRANSLATION2D = new Translation2d(0.381, 0.381);
    /**
     * Translation2d (Location) of Front Right motor (x, y). Measured in meters.
     */
    public static final Translation2d FRONTRIGHT_TRANSLATION2D = new Translation2d(0.381, -0.381);
    /**
     * Translation2d (Location) of Back Left motor (x, y). Measured in meters.
     */
    public static final Translation2d BACKLEFT_TRANSLATION2D = new Translation2d(-0.381, 0.381);
    /**
     * Translation2d (Location) of Back Right motor (x, y). Measured in meters.
     */
    public static final Translation2d BACKRIGHT_TRANSLATION2D = new Translation2d(-0.381, -0.381);

    /**
     * Trackwidth used for a DifferentialDriveSystem
     */
    public static final double DIFFERENTIAL_TRACK_WIDTH = 27.0d; // measured in inches

    /**
     * Minimum drive speed in ControlPercent form (-1.0 to 1.0 with 0.0 being at rest)
     */
    public static final double MIN_DRIVE_DUTYCYCLE = 0.05d;

    /**
     * Minimum speed of rotate motor (swerve only) in ControlPercent form (-1.0 to 1.0 with 0.0 being at rest)
     */
    public static final double MIN_SWERVE_ROTMOTOR_DUTYCYCLE = 0.05d;

    /**
     * PID Tolerance for SwerveModule rotate motor
     */
    public static final double SWERVE_MODULE_TOLERANCE = 0.1;

    /**
     * Max Drive ControlPercent (suggested to leave at 1.0)
     */
    public static final double MAX_CONTROLPERCENT = 1.0d;

    /**
     * Minimum ControlPercent RotationSpeed
     */
    public static final double MIN_ROTSPEED_DUTYCYCLE = 0.1d;

    /**
     * Max voltage of battery in order to enable MotorController battery compensation 
     */
    public static final double MAX_VOLTAGE = 12.0d;

    /**
     * Everytime a drive motor encoder position is multiplied by this, it should return the distance the wheel has travelled.
     */
    public static final double ENC_TO_METERS_FACTOR = 0.000004d;

    /**
     * Everytime a rotate motor encoder position is multiplied by this (encoder conversion factor defaults to radians), it will return raw encoder counts.
     */
    public static final double RAD_TO_ENC_FACTOR = 0.00004d;

    /**
     * 2π 
     */
    public static final double TWO_PI = Math.PI*2;

    /**
     * Max velocity we allow the DriveSystem to travel in meters per second
     */
    public static final double MAX_VELOCITY = 3.5d;
    /**
     * Max Y/Strafe velocity we allow the DriveSystem to travel in meters per second.
     */
    public static final double MAX_STRAFE_VELOCITY = 3.5d;
    /**
     * Max Angular Velocity in degrees per second.
     * You may change this value.
     */
    public static final double MAX_ANGULAR_VELOCITY_DEG = 90d;

    /**
     * Minimum drive velocity (meters per second)
     */
    public static final double MIN_DRIVE_VELOCITY = 0.75d;

    /**
     * Minimum RotateSpeed (Radians per second)
     */
    public static final double MIN_ROTSPEED = Math.toRadians(1);

    /**
     * Gets the Max Angular Velocity (deg) that is set by the user and converts it to radians.
     * Do not change this value.
     */
    public static final double MAX_ANGULAR_VELOCITY = Math.toRadians(MAX_ANGULAR_VELOCITY_DEG);

    /**
     * Drive Translation Porportional gain. PID input is motor velocity and the output is X (or Y) position (meters).
     */
    public static final double ROBOT_POSE_P = 0.0d;
    /**
     * Drive Translation Integral gain (not recommended to be used).
     */
    public static final double ROBOT_POSE_I = 0.0d;
    /**
     * Drive Translation Derivative gain.
     */
    public static final double ROBOT_POSE_D = 0.0d;
    /**
     * Drive Translation FeedForward gain.
     */
    public static final double ROBOT_POSE_FF = 0.0d;


    /**
     * Drive Translation Porportional gain. PID input is motor control percent speed and the output is motor encoder velocity (with conversion).
     */
    public static final double ROBOT_VELOCITY_P = 0.0d;
    /**
     * Drive Translation Integral gain (not recommended to be used).
     */
    public static final double ROBOT_VELOCITY_I = 0.0d;
    /**
     * Drive Translation Derivative gain.
     */
    public static final double ROBOT_VELOCITY_D = 0.0d;
    /**
     * Drive Translation FeedForward gain.
     */
    public static final double ROBOT_VELOCITY_FF = 1023 / (MAX_VELOCITY / ENC_TO_METERS_FACTOR);

    /**
     * SwerveModule Rotate Motor Porportional gain. PID input is swerve rotate motor control percent speed and the output is rotate motor encoder postion,
     * which defaults to radians based on how much someone messes with base SwerveModule.java
     */
    public static final double SWERVE_ROT_P = 0.0d;
    /**
     * SwerveModule RotateMotor Integral gain (not recommended to be used).
     */
    public static final double SWERVE_ROT_I = 0.0d;
    /**
     * SwerveModule RotateMotor Derivative gain.
     */
    public static final double SWERVE_ROT_D = 0.0d;
    /**
     * SwerveModule RotateMotor Feedforward gain (usually left at 0.0).
     */
    public static final double SWERVE_ROT_FF = 0.0d;

    /**
     * Robot spin Porportional gain. PID input is input rotate speed in radians per second and the output is the gyro angle in radians.
     */
    public static final double ROBOT_ROT_P = 0.0d;
    /**
     * Robot spin Integral gain. (Not recommended to be used)
     */
    public static final double ROBOT_ROT_I = 0.0d;
    /**
     * Robot spin Derivative gain.
     */
    public static final double ROBOT_ROT_D = 0.0d;
    /**
     * Robot spin PIDController tolerance
     */
    public static final double ROBOT_ROT_TOLERANCE = Math.toRadians(0.5);

    
    public static final HashMap<String, Command> markerMap = new HashMap<>();


    public static final void addPathEventMarkers() {
        markerMap.put("marker1", new PrintCommand("Passed marker 1"));
    }
}
