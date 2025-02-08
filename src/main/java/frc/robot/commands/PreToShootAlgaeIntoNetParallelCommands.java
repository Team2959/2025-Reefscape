// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.liftTargetPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreToShootAlgaeIntoNetParallelCommands extends ParallelCommandGroup {
  /** Creates a new PreToShootAlgaeIntoNet. */
  public PreToShootAlgaeIntoNetParallelCommands(LiftSubsystem liftSubsystem, AlgaeClawSubsystem algaeSubsystem)
  {
    addCommands(new LiftDriveToPositionCommand(liftSubsystem, liftTargetPositions.L4),
      new InstantCommand(() -> algaeSubsystem.extendSolenoid()),
      // should wait for velocity yo be reached, but it should be much faster that lift to L4
      new InstantCommand(() -> algaeSubsystem.setClawShootSpeed(), algaeSubsystem));
  }
}
