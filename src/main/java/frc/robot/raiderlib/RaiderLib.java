package frc.robot.raiderlib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.raiderlib.builders.MovingPart;
import frc.robot.raiderlib.builders.SwerveDriveSystem;
import frc.robot.raiderlib.drive.DriveSystem;

/**
 * Subsystem that handles MovingParts as well as the ability to easily
 * switch between Differential, Mecanum, and Swerve drive systems without
 * needing to change anything more than a few Constants. This Robot Code template
 * is very useful in helping guarantee the sucess of the idea that Team 1518 will continue
 * to progress on the software side of things as this project makes using
 * advanced control (aka PIDControl) very simple.
 * @author Chloe Quinn
 * @version 1.0
 */
public class RaiderLib extends SubsystemBase{
    
    /**
     * Create a list of MovingParts
     */
    private final ArrayList<MovingPart> movingParts;
    /**
     * Create a DriveSystem object
     */
    private DriveSystem driveSystem;
     /**
      * Create an instance object for easier access to class fields.
      */
    public static RaiderLib INSTANCE;

    /**
     * RaiderLib Initializer
     */
    public RaiderLib() {
        movingParts = new ArrayList<>();
        driveSystem = new SwerveDriveSystem(new XboxController(0));
        INSTANCE = this;
    }

    @Override
    public void periodic() {
        for(MovingPart part : movingParts) {
            part.periodic();
        }
        if(driveSystem != null) {
            driveSystem.periodic();
        }
    }

    /**
     * Set the current drive system
     * @param driveSystem DriveSystem object
     */
    public void setDriveSystem(DriveSystem driveSystem) {
        this.driveSystem = driveSystem;
    }

    /**
     * 
     * @return DriveSystem as an object
     */
    public DriveSystem getDriveSystem() {
        return this.driveSystem;
    }

    /**
     * Add a MovingPart to the RaiderLib Subsystem that will be ran periodically.
     * @param part MovingPart object
     */
    public void addPart(MovingPart part) {
        movingParts.add(part);
        part.onAddToList();
    }
}
