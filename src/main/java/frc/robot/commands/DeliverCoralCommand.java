// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralControlTargetSpeeds;
import frc.robot.subsystems.LiftSubsystem.liftTargetLevels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeliverCoralCommand extends WaitCommand {
  /** Creates a new DeliverCoralCommand. */
  private CoralDeliverySubsystem m_coralDeliverySubsystem;
  private CoralControlTargetSpeeds m_targetLeftSpeed;
  private CoralControlTargetSpeeds m_targetRightSpeed;
  private LiftSubsystem m_LiftSubsystem;

  public DeliverCoralCommand(
    double seconds,
    CoralDeliverySubsystem coralDeliverySubsystem,
    CoralControlTargetSpeeds coralControlLeftTargetSpeed,
    CoralControlTargetSpeeds coralControlRightTargetSpeed,
    LiftSubsystem liftSubsystem)
  {
    super(seconds);
    m_coralDeliverySubsystem = coralDeliverySubsystem;
    m_targetLeftSpeed = coralControlLeftTargetSpeed;
    m_targetRightSpeed = coralControlRightTargetSpeed;
    m_LiftSubsystem = liftSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_coralDeliverySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    super.initialize();
    var isAboveL3 = m_LiftSubsystem.isAbovePosition(liftTargetLevels.L3);
    m_coralDeliverySubsystem.setLeftCoralControlVelocity(
      isAboveL3 ? CoralControlTargetSpeeds.FeedL4 : m_targetLeftSpeed);
    m_coralDeliverySubsystem.setRightCoralControlVelocity(
      isAboveL3 ? CoralControlTargetSpeeds.FeedL4 : m_targetRightSpeed);
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
