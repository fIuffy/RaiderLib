package frc.robot.raiderlib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.raiderlib.builders.MovingPart;
import frc.robot.raiderlib.builders.SwerveDriveSystem;
import frc.robot.raiderlib.drive.DriveSystem;
import frc.robot.raiderlib.drive.DriveSystem.DriveExportMode;

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


    public static SendableChooser<Command> driveSetupChooser = new SendableChooser<Command>();

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
        setupDashboard();
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

    public ArrayList<MovingPart> getMovingParts() {
        return this.movingParts;
    }

    public Command getAutoDriveCommand(String pathName) {
        return driveSystem.fullAuto(pathName);
    }
    
    private void setupDashboard() { 
        driveSetupChooser.addOption("VelocityExport", driveSystem.exportPIDData(DriveExportMode.VELOCITY));
        driveSetupChooser.addOption("SpinExport", driveSystem.exportPIDData(DriveExportMode.SPIN));
        driveSetupChooser.addOption("PoseExport", driveSystem.exportPIDData(DriveExportMode.POSITION));
        driveSetupChooser.addOption("ModuleRotExport", driveSystem.exportPIDData(DriveExportMode.MODULESPIN)); // Swerve only
        SmartDashboard.putData("Run DriveSetup", Commands.sequence(driveSetupChooser.getSelected()));
        SmartDashboard.putData(driveSetupChooser);
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
