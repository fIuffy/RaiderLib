// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.raiderlib.RaiderLib;

public class RobotContainer {

  /** 
   * Create the RaiderLib Subsystem object
   */ 
  public static final RaiderLib raiderLib = new RaiderLib();

  public static SendableChooser<String> autoChooser = new SendableChooser<String>();
  public static SendableChooser<Command> driveSetupChooser = new SendableChooser<Command>();

  public RobotContainer() {
      configureAuto();
      configureBindings();
      configureSetup();
  }

  private void configureBindings() {
    
  }

  private void configureAuto() {
      autoChooser.setDefaultOption("Wait 1 sec(do nothing)", "wait");
      autoChooser.addOption("Forward 1m", "example");
      autoChooser.addOption("Snake", "example2");
      autoChooser.addOption("SnakeSpin", "example3");
      SmartDashboard.putData(autoChooser);
  }

  private void configureSetup() {
      driveSetupChooser.addOption("VelocityExport", raiderLib.driveExportVelocity());
      driveSetupChooser.addOption("SpinExport", raiderLib.driveExportSpin());
      driveSetupChooser.addOption("PoseExport", raiderLib.driveExportPose());
      driveSetupChooser.addOption("OneModuleRotExport", raiderLib.driveExportModuleRot()); // Swerve only
      SmartDashboard.putData("Run DriveSetup", Commands.sequence(driveSetupChooser.getSelected()));
      SmartDashboard.putData(driveSetupChooser);
  }

  public Command getAutonomousCommand() {
      return raiderLib.getAutoDriveCommand(autoChooser.getSelected());
  }
}
