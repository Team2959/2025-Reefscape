// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIndexSubsystem;
import frc.robot.subsystems.CoralIndexSubsystem.CoralIndexTargetPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexCoralCommand extends Command {
  /** Creates a new IndexCoralCommand. */
  private CoralIndexSubsystem m_coralIndexSubsystem;
  private CoralIndexTargetPositions m_target;
  public IndexCoralCommand(CoralIndexSubsystem coralIndexSubsystem, CoralIndexTargetPositions target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coralIndexSubsystem = coralIndexSubsystem;
    m_target = target;
    addRequirements(m_coralIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralIndexSubsystem.setTargetIndexPosition(m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      m_coralIndexSubsystem.stopAtIndexCurrentPosition();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coralIndexSubsystem.isAtIndexTargetPosition();
  }
}
