// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.cwtech.AprilTagPID;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralControlTargetSpeeds;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralIndexTargetPositions;
import frc.robot.subsystems.LiftSubsystem.liftTargetPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralPlacementSequentialCommand extends SequentialCommandGroup {
  /** Creates a new CoralPlacementSequentialCommand. */
  public CoralPlacementSequentialCommand(
    LiftSubsystem liftSubsystem,
    DriveSubsystem drivesubsystem,
    CoralDeliverySubsystem coralDeliverySubsystem,
    liftTargetPositions targetLevel,
    AprilTagPID aprilTagPID 
    )
  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignWithReefCommand(drivesubsystem, aprilTagPID),
      new LockWheelsCommand(drivesubsystem).alongWith(new LiftDriveToPositionCommand(liftSubsystem, targetLevel)),
      new DeliverCoralCommand(CoralDeliverySubsystem.DeliveryWaitSeconds, coralDeliverySubsystem, CoralControlTargetSpeeds.Feed, CoralControlTargetSpeeds.Feed),
      new LiftDriveToPositionCommand(liftSubsystem, liftTargetPositions.Base).alongWith(new IndexCoralCommand(coralDeliverySubsystem,CoralIndexTargetPositions.Center))
    );
  }
}
