package frc.robot.raiderlib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.raiderlib.builders.MovingPart;
import frc.robot.raiderlib.drive.DriveSystem;

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
        driveSystem = null;
        INSTANCE = this;
    }

    @Override
    public void periodic() {
        for(MovingPart part : movingParts) {
            part.periodic();
        }
        driveSystem.periodic();
    }

    public void setDriveSystem(DriveSystem driveSystem) {
        this.driveSystem = driveSystem;
    }

    public DriveSystem getDriveSystem() {
        return this.driveSystem;
    }

    public void addPart(MovingPart part) {
        movingParts.add(part);
        part.onAddToList();
    }
}
