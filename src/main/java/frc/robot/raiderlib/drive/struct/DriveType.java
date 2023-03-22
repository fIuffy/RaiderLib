package frc.robot.raiderlib.drive.struct;

public enum DriveType {
    TWO_MOTOR_BASIC(DriveConstants.LEFT, DriveConstants.RIGHT),
    TWO_MOTOR_MECANUM(DriveConstants.LEFT, DriveConstants.RIGHT),
    FOUR_MOTOR_BASIC(DriveConstants.LEFT_FRONT, DriveConstants.LEFT_REAR, DriveConstants.RIGHT_REAR, DriveConstants.RIGHT_FRONT),
    FOUR_MOTOR_MECANUM(DriveConstants.LEFT_FRONT, DriveConstants.LEFT_REAR, DriveConstants.RIGHT_REAR, DriveConstants.RIGHT_FRONT),

    SWERVE(DriveConstants.FRONT_LEFT_MOVE_MOTOR, DriveConstants.FRONT_LEFT_ROTATE_MOTOR, DriveConstants.FRONT_LEFT_ROTATE_SENSOR,
        DriveConstants.REAR_LEFT_MOVE_MOTOR, DriveConstants.REAR_LEFT_ROTATE_MOTOR, DriveConstants.REAR_LEFT_ROTATE_SENSOR,
        DriveConstants.REAR_RIGHT_MOVE_MOTOR, DriveConstants.REAR_RIGHT_ROTATE_MOTOR, DriveConstants.REAR_RIGHT_ROTATE_SENSOR,
        DriveConstants.FRONT_RIGHT_MOVE_MOTOR, DriveConstants.FRONT_RIGHT_ROTATE_MOTOR, DriveConstants.FRONT_RIGHT_ROTATE_SENSOR);

    private final int[] idList;
    private DriveType(int... idList) {
        this.idList = idList;
    }

    public int[] getIdList() {
        return this.idList;
    }
}
