// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerveDrive = new SwerveSubsystem();

  // Driver controller instance
  private final CommandXboxController driverController = new CommandXboxController(0);

  
  public RobotContainer() 
  {
    configureBindings();

    swerveDrive.setDefaultCommand
                                (new RunCommand
                                (()-> swerveDrive.driveFieldOperated(
                                  ()-> -driverController.getLeftY(), 
                                  ()-> -driverController.getLeftX(), 
                                  ()-> -driverController.getRightX()),
                                  swerveDrive));
  }

  private void configureBindings() 
  {
  }

  public Command getAutonomousCommand() 
  {
    return null;
  }
}
