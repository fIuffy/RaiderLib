package frc.robot.raiderlib.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.raiderlib.drive.struct.DriveType;

public class DriveSystem {
    private final DriveType driveType;
    private final XboxController controller;
    private final AHRS gyro;

    public DriveSystem(DriveType driveType, XboxController controller) {
        this.driveType = driveType;
        this.controller = controller;
        this.gyro = new AHRS(SerialPort.Port.kUSB);
    }


    public void periodic() {

    }


    /**
     * Move the drive system using PID for error correction
     * @param meters
     * @param nonHolonomicAngle
     */
    public void move(double meters, double nonHolonomicAngle) {

    }


    public DriveType getDriveType() {
        return this.driveType;
    }

    public XboxController getController() {
        return this.controller;
    }

    public AHRS getGyro() {
        return this.gyro;
    }


}
