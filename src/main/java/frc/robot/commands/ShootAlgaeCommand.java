// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAlgaeCommand extends WaitCommand {
  /** Creates a new ShootAlgaeIntoProcessorCommand. */
  private AlgaeClawSubsystem m_algaeClawSubsystem;

  public ShootAlgaeCommand(AlgaeClawSubsystem algaeClawSubsystem) {
    super(1);
    algaeClawSubsystem = m_algaeClawSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algaeClawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_algaeClawSubsystem.feedAlgaeIntoProcessor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeClawSubsystem.stopClawShootMotor();
    m_algaeClawSubsystem.stopClawIntakeMotor();
  }
}
