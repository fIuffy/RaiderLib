package frc.robot.raiderlib.drive.struct;

public class DriveConstants {
    /**
     * Two Motor
     */
        public static final int LEFT = -1;
        public static final int RIGHT = -1;

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
     * Spin PID
     */
    public static final double ROBOT_SPIN_P = 0.0d;
    public static final double ROBOT_SPIN_I = 0.0d;
    public static final double ROBOT_SPIN_D = 0.0d;
}
