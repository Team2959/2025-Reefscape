// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.liftTargetLevels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftMoveToLevelCommand extends Command {
  /** Creates a new LiftDriveToPositionCommand. */
  private LiftSubsystem m_LiftSubsystem;
  private liftTargetLevels m_targetPosition;

  public LiftMoveToLevelCommand(LiftSubsystem liftSubsystem, liftTargetLevels targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_targetPosition = targetPosition;
    m_LiftSubsystem = liftSubsystem;
    addRequirements(m_LiftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LiftSubsystem.setTargetPosition(m_targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LiftSubsystem.comparePrimaryEncodertoAlternateEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if (interrupted)
      m_LiftSubsystem.stopAtCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_LiftSubsystem.isAtTargetPosition();
  }
}
