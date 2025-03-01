// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.cwtech.AprilTagPID;
import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.liftTargetLevels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeReefRemovelSequentialCommand extends SequentialCommandGroup {
  /** Creates a new AlgaeReefRemovelSequentialCommand. */
  public AlgaeReefRemovelSequentialCommand(
    LiftSubsystem liftSubsystem,
    liftTargetLevels targetLevel,
    DriveSubsystem driveSubsystem,
    AprilTagPID aprilTagPID,
    AlgaeClawSubsystem algaeClawSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignWithReefCommand(driveSubsystem, aprilTagPID),
      new LiftMoveToLevelCommand(liftSubsystem, targetLevel).alongWith(new IntakeAlgaeCommand(algaeClawSubsystem)),
      new LiftMoveToLevelCommand(liftSubsystem, liftTargetLevels.Base)
    );
  }
}
