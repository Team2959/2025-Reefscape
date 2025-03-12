// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.cwtech.AprilTagPID;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithReefCommand extends Command {
  /** Creates a new AlignWithReefCommand. */
  private DriveSubsystem m_driveSubsystem;
  private final AprilTagPID m_AprilTagPID;
  private double m_targetZ = 0.48; 
  private double m_targetRotation;

  public AlignWithReefCommand(DriveSubsystem driveSubsystem, AprilTagPID aprilTagPID) {
    m_driveSubsystem = driveSubsystem;
    m_AprilTagPID = aprilTagPID;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var tid = (int)LimelightHelpers.getFiducialID("limelight");
    if (tid == 9 || tid == 22)
      m_targetRotation = 60;
    else if (tid == 8 || tid == 17)
      m_targetRotation = 120;
    else if (tid == 7 || tid == 18)
      m_targetRotation = 180;
    else if (tid == 6 || tid == 19)
      m_targetRotation = 240;
    else if (tid == 11 || tid == 20)
      m_targetRotation = 300;
    else
      m_targetRotation = 0;
    
    m_AprilTagPID.setTargetPosition(0, m_targetZ, m_targetRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AprilTagPID.driveToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
