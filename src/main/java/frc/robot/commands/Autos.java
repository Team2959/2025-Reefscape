// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralControlTargetSpeeds;
import frc.robot.subsystems.CoralIndexSubsystem.CoralIndexTargetPositions;
import frc.robot.subsystems.LiftSubsystem.liftTargetLevels;

/** Add your docs here. */
public final class Autos {

    public static void registerPathPlannerNamedCommands(RobotContainer robotContainer)
    {
        var container = robotContainer;
        NamedCommands.registerCommand("Index Coral Right", new IndexCoralCommand(container.m_coralIndexSubsystem, CoralIndexTargetPositions.Right));
        NamedCommands.registerCommand("Place at Trough Right", new DeliverCoralCommand(container.m_coralDeliverySubsystem.m_deliveryWaitSeconds, container.m_coralDeliverySubsystem, CoralControlTargetSpeeds.L1FastSpeed, CoralControlTargetSpeeds.L1SlowSpeed));
        // NamedCommands.registerCommand("Place at L4", new CoralPlacementSequentialCommand(container.m_liftSubsystem, container.m_driveSubsystem, container.m_coralDeliverySubsystem, liftTargetPositions.L4, container.m_aprilTagPID));
        // NamedCommands.registerCommand("Intake Coral from Wall", new IntakeCoralCommand(container.m_coralDeliverySubsystem));
        // NamedCommands.registerCommand("Low Reef Algae Removal", new AlgaeReefRemovelSequentialCommand(container.m_liftSubsystem, liftTargetPositions.L2, container.m_driveSubsystem, container.m_aprilTagPID, container.m_algaeClawSubsystem));
        // NamedCommands.registerCommand("High Reef Algae Removal", new AlgaeReefRemovelSequentialCommand(container.m_liftSubsystem, liftTargetPositions.L3, container.m_driveSubsystem, container.m_aprilTagPID, container.m_algaeClawSubsystem));
    }
}
