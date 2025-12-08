// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveDefault;
import frc.robot.commands.Intake.ElevatorDefault;
import frc.robot.commands.Intake.FistDefault;
import frc.robot.commands.Intake.TelescopeDefault;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Intake.ElevatorSubsystem;
import frc.robot.subsystems.Intake.FistSubsystem;
import frc.robot.subsystems.Intake.PneumaticSubsystem;
import frc.robot.subsystems.Intake.TelescopeSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  public static final DriveDefault driveDefault = new DriveDefault(driveTrain);

  public static final ElevatorSubsystem eSubsystem = new ElevatorSubsystem();
  public static final ElevatorDefault eCommand = new ElevatorDefault(eSubsystem);

  public static final TelescopeSubsystem tSubsystem = new TelescopeSubsystem();
  public static final TelescopeDefault tCommand = new TelescopeDefault(tSubsystem); 
  
  public static final FistSubsystem fSubsystem = new FistSubsystem();
  public static final FistDefault fCommand  = new FistDefault(fSubsystem);

  public static final PneumaticSubsystem pcmSubsystem = new PneumaticSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);

  public static final Joystick m_operatorController =
      new Joystick(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveDefault.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveDefault);

    eCommand.addRequirements(eSubsystem);
    eSubsystem.setDefaultCommand(eCommand);

    tCommand.addRequirements(tSubsystem);
    tSubsystem.setDefaultCommand(tCommand);

    fCommand.addRequirements(fSubsystem);
    fSubsystem.setDefaultCommand(fCommand);

    pcmSubsystem.startComp();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    JoystickButton closeLT = new JoystickButton(m_driverController, 7);
    closeLT.whileTrue(new RunCommand(()-> pcmSubsystem.closeIntake()));

    JoystickButton openRT = new JoystickButton(m_driverController, 8); 
    openRT.whileTrue(new RunCommand(()-> pcmSubsystem.openIntake()));

    JoystickButton avancoPidX = new JoystickButton(m_driverController, 1);
    avancoPidX.whileTrue(new RunCommand(()-> tSubsystem.telescopeSetpoint(-200)).alongWith(
                                        new RunCommand(()-> fSubsystem.fistSetpoint(-260))));

    JoystickButton homePidA = new JoystickButton(m_driverController, 2);
    homePidA.whileTrue(new RunCommand(()-> tSubsystem.telescopeSetpoint(0)).alongWith(
                                        new RunCommand(()-> fSubsystem.fistSetpoint(0))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(driveTrain);
  }
}
