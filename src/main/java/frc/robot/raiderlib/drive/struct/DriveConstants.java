package frc.robot.raiderlib.drive.struct;

import edu.wpi.first.math.geometry.Translation2d;

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
    

    // Locations of the wheels relative to the robot center. (Measured in meters)
    public static final Translation2d FRONTLEFTT_TRANSLATION2D = new Translation2d(0.381, 0.381);
    public static final Translation2d FRONTRIGHT_TRANSLATION2D = new Translation2d(0.381, -0.381);
    public static final Translation2d BACKLEFT_TRANSLATION2D = new Translation2d(-0.381, 0.381);
    public static final Translation2d BACKRIGHT_TRANSLATION2D = new Translation2d(-0.381, -0.381);

    public static final double DIFFERENTIAL_TRACK_WIDTH = 27.0d; // measured in inches

    public static final double MIN_DRIVE_DUTYCYCLE = 0.05d;

    public static final double MIN_SWERVE_ROTMOTOR_DUTYCYCLE = 0.05d;

    public static final double SWERVE_MODULE_TOLERANCE = 0.1;

    public static final double MAX_CONTROLPERCENT = 1.0d;

    public static final double MAX_VOLTAGE = 12.0d;

    public static final double ENC_TO_METERS_FACTOR = 0.000004d;
    public static final double RAD_TO_ENC_FACTOR = 0.00004d;

    public static final double TWO_PI = Math.PI*2;

    public static final double MAX_VELOCITY = 3.5d; // meters per second
    public static final double MIN_DRIVE_VELOCITY = 1.0d;
    public static final double MAX_ANGULAR_VELOCITY_DEG = 180d; // degrees per second (CHANGE THIS ONE)
    public static final double MAX_ANGULAR_VELOCITY = MAX_ANGULAR_VELOCITY_DEG*(Math.PI/180d); // radians per second (DONT CHANGE THIS ONE)

    /**
     * Drive Translation PID
     * PIDController Details:
     *  Input: Drive Motor ControlPercent Speed
     *  Output: Drive Motor Encoder Velocity (with conversion)
     */
    public static final double ROBOT_TRANSLATION_P = 0.0d;
    public static final double ROBOT_TRANSLATION_I = 0.0d;
    public static final double ROBOT_TRANSLATION_D = 0.0d;
    public static final double ROBOT_TRANSLATION_FF = 1023 / (MAX_VELOCITY / ENC_TO_METERS_FACTOR);

    /**
     * Swerve Rotate Motor PID
     * PIDController Details:
     *  Input: Rotate Motor ControlPercent Speed
     *  Output: Rotate Motor Encoder Position (defaults to be in radians)
     */
    public static final double SWERVE_ROT_P = 0.0d;
    public static final double SWERVE_ROT_I = 0.0d;
    public static final double SWERVE_ROT_D = 0.0d;
    public static final double SWERVE_ROT_FF = 0.0d;

    /**
     * Robot Spin PID
     * PIDController Details:
     *  Input: RotSpeed
     *  Output: Continuous Gyro Angle
     */
    public static final double ROBOT_ROT_P = 0.0d;
    public static final double ROBOT_ROT_I = 0.0d;
    public static final double ROBOT_ROT_D = 0.0d;
    public static final double ROBOT_ROT_TOLERANCE = Math.toRadians(0.5);
    public static final double MIN_ROT_OUTPUT = 0.10;
}
