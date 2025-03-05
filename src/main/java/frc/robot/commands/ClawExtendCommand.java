// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.liftTargetLevels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawExtendCommand extends Command {
  /** Creates a new ClawExtendCommand. */
private LiftSubsystem m_liftsSubsystem;
private AlgaeClawSubsystem m_AlgaeClawSubsystem;
private liftTargetLevels m_targetPosition;

  public ClawExtendCommand(
    LiftSubsystem liftSubsystem,
    AlgaeClawSubsystem algaeClawSubsystem,
    liftTargetLevels targetPosition
  ) { m_liftsSubsystem = liftSubsystem;
      m_AlgaeClawSubsystem = algaeClawSubsystem;
      m_targetPosition = targetPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_AlgaeClawSubsystem);
    addRequirements(m_liftsSubsystem);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_liftsSubsystem.setTargetPosition(liftTargetLevels.LowAlage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_liftsSubsystem.
    // m_AlgaeClawSubsystem.intakeAlgae();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeClawSubsystem.stopClawWheels();
    m_liftsSubsystem.stopAtCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
