// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCommand extends Command {
  /** Creates a new IntakeAlgaeCommand. */
  AlgaeClawSubsystem m_algaeClawSubsystem;
  public IntakeAlgaeCommand(AlgaeClawSubsystem algaeClawSubsystem) {
    algaeClawSubsystem = m_algaeClawSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algaeClawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algaeClawSubsystem.intakeAlgae();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeClawSubsystem.stopClawIntakeMotor();
    m_algaeClawSubsystem.stopClawShootMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
