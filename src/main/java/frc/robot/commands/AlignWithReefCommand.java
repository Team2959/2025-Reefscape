// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  private final DoublePublisher m_alignmentProgressPub;
  private final BooleanPublisher m_alignmentCompletePub;

  //constants for progress calculation
  private static final double MAX_ROTATION_ERROR = 10.0; //degrees
  private static final double MAX_POSITION_ERROR = 1.0; //units

  public AlignWithReefCommand(DriveSubsystem driveSubsystem, AprilTagPID aprilTagPID) {
    m_driveSubsystem = driveSubsystem;
    m_AprilTagPID = aprilTagPID;
    // Use addRequirements() here to declare subsystem dependencies.

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("ReefAlignment");

    var progressPub = table.getDoubleTopic("progress");
    m_alignmentProgressPub = progressPub.publish();
    m_alignmentProgressPub.set(0.0);

    var completePub = table.getBooleanTopic("Complete");
    m_alignmentCompletePub = completePub.publish();
    m_alignmentCompletePub.set(false);

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

    //Reset Dashboard Values
    m_alignmentProgressPub.set(0);
    m_alignmentCompletePub.set(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    if (robotSpace != null && robotSpace.length >= 3) {
      m_AprilTagPID.driveToTarget();
      
      // Calculate alignment progress and update dashboard
      updateDashboard();
    } else {
      // If no AprilTag data is available, stop the robot and show 0% progress
      m_driveSubsystem.drive(0, 0, 0, true);
      m_alignmentProgressPub.set(0.0);
      m_alignmentCompletePub.set(false);
    }
  }

  private void updateDashboard() {
    boolean isComplete = m_AprilTagPID.atTargetPosition();
    m_alignmentCompletePub.set(isComplete);
    
    // Get the current errors
    double deltaX = m_AprilTagPID.getDeltaX();
    double deltaZ = m_AprilTagPID.getDeltaZ();
    double deltaRotation = m_AprilTagPID.getDeltaRotation();
    
    // Ensure the PID controller has valid AprilTag data
    double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    if (robotSpace == null || robotSpace.length < 3) {
      // Invalid data, return early without updating progress
      return;
    }
    
    // Calculate the total distance from target (Euclidean distance in 3D space)
    // Scale rotation by a factor to make it comparable to position values
    double rotationScale = MAX_POSITION_ERROR / MAX_ROTATION_ERROR;
    double scaledRotation = deltaRotation * rotationScale;
    
    // Calculate the total distance using 3D Euclidean distance formula
    double totalDistance = Math.sqrt(deltaX * deltaX + deltaZ * deltaZ + scaledRotation * scaledRotation);
    
    // Calculate maximum possible distance for normalization
    double maxDistance = Math.sqrt(
        MAX_POSITION_ERROR * MAX_POSITION_ERROR + 
        MAX_POSITION_ERROR * MAX_POSITION_ERROR + 
        (MAX_ROTATION_ERROR * rotationScale) * (MAX_ROTATION_ERROR * rotationScale)
    );
    
    // Calculate progress as percentage (inverse of distance)
    double progressPercentage = Math.max(0, Math.min(100, (1.0 - totalDistance / maxDistance) * 100));
    
    // If we're complete, progress is 100%
    if (isComplete) {
      progressPercentage = 100.0;
    }
    
    m_alignmentProgressPub.set(progressPercentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true);

    m_alignmentCompletePub.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
