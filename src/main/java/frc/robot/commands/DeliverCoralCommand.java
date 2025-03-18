// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralControlTargetSpeeds;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeliverCoralCommand extends WaitCommand {
  /** Creates a new DeliverCoralCommand. */
  private CoralDeliverySubsystem m_coralDeliverySubsystem;
  private CoralControlTargetSpeeds m_targetSpeed;

  public DeliverCoralCommand(
    double seconds,
    CoralDeliverySubsystem coralDeliverySubsystem,
    CoralControlTargetSpeeds coralControlTargetSpeed)
  {
    super(seconds);
    m_coralDeliverySubsystem = coralDeliverySubsystem;
    m_targetSpeed = coralControlTargetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_coralDeliverySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    super.initialize();
    m_coralDeliverySubsystem.setCoralControlVelocity(m_targetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_coralDeliverySubsystem.stopCoralControlMotor();
    m_coralDeliverySubsystem.setCoralPresent(false);
  }
}
