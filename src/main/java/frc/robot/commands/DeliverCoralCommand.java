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
  private CoralControlTargetSpeeds m_targetLeftSpeed;
  private CoralControlTargetSpeeds m_targetRightSpeed;

  public DeliverCoralCommand(double seconds, CoralDeliverySubsystem coralDeliverySubsystem, CoralControlTargetSpeeds coralControlLeftTargetSpeed, CoralControlTargetSpeeds coralControlRightTargetSpeed)
  {
    super(seconds);
    m_coralDeliverySubsystem = coralDeliverySubsystem;
    m_targetLeftSpeed = coralControlLeftTargetSpeed;
    m_targetRightSpeed = coralControlRightTargetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_coralDeliverySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    super.initialize();
    m_coralDeliverySubsystem.setLeftCoralControlVelocity(m_targetLeftSpeed);
    m_coralDeliverySubsystem.setRightCoralControlVelocity(m_targetRightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_coralDeliverySubsystem.stopLeftCoralControlMotor();
    m_coralDeliverySubsystem.stopRightCoralControlMotor();
    m_coralDeliverySubsystem.setCoralPresent(false);
  }
}
