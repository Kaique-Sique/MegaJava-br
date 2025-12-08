package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake.TelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TelescopeDefault extends CommandBase {
 
  private final TelescopeSubsystem tSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TelescopeDefault(TelescopeSubsystem subsystem) {
    tSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.tSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tSubsystem.telescopeWithJoystick(RobotContainer.m_operatorController, OperatorConstants.speedTelescope);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}