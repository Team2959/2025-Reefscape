// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralControlTargetSpeeds;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralCommand extends Command {
  /** Creates a new IntakeCoralCommand. */
  private CoralDeliverySubsystem m_coralDeliverySubsystem;
  public IntakeCoralCommand(CoralDeliverySubsystem coralDeliverySubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coralDeliverySubsystem = coralDeliverySubsystem;
    addRequirements(m_coralDeliverySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralDeliverySubsystem.setCoralPresent(false);
    m_coralDeliverySubsystem.setRightCoralControlVelocity(CoralControlTargetSpeeds.Intake);
    m_coralDeliverySubsystem.setLeftCoralControlVelocity(CoralControlTargetSpeeds.Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_coralDeliverySubsystem.getOpticSensor()){
      m_coralDeliverySubsystem.setCoralPresent(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralDeliverySubsystem.stopLeftCoralControlMotor();
    m_coralDeliverySubsystem.stopRightCoralControlMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coralDeliverySubsystem.isCoralInPosition();
  }
}
