// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.raiderlib.RaiderLib;

public class RobotContainer {

  /** 
   * Create the RaiderLib Subsystem object
   */ 
  public static RaiderLib raiderLib;

  public static SendableChooser<String> autoChooser;
  

  public RobotContainer() {
        autoChooser = new SendableChooser<String>();
        configureAuto();
        configureBindings();
        raiderLib = new RaiderLib();
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

  public Command getAutonomousCommand() {
        if(raiderLib == null || autoChooser == null || autoChooser.getSelected() == null) return new PrintCommand("Robot Code fields not initialized yet.");
        return raiderLib.getAutoDriveCommand(autoChooser.getSelected());
  }
}
