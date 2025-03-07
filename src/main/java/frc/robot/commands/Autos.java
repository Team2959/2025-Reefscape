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
        NamedCommands.registerCommand("Index Coral Left", new IndexCoralCommand(container.m_coralIndexSubsystem, CoralIndexTargetPositions.Left));
        NamedCommands.registerCommand("Index Coral Center", new IndexCoralCommand(container.m_coralIndexSubsystem, CoralIndexTargetPositions.Center));
        NamedCommands.registerCommand("Place at Trough Right", new DeliverCoralCommand(container.m_coralDeliverySubsystem.m_deliveryWaitSeconds, container.m_coralDeliverySubsystem, CoralControlTargetSpeeds.L1FastSpeed, CoralControlTargetSpeeds.L1SlowSpeed, container.m_liftSubsystem));
        NamedCommands.registerCommand("Place at Trough Left", new DeliverCoralCommand(container.m_coralDeliverySubsystem.m_deliveryWaitSeconds, container.m_coralDeliverySubsystem, CoralControlTargetSpeeds.L1SlowSpeed, CoralControlTargetSpeeds.L1FastSpeed, container.m_liftSubsystem));
        NamedCommands.registerCommand("Deliver Coral",
            new DeliverCoralCommand(container.m_coralDeliverySubsystem. m_deliveryWaitSeconds, container.m_coralDeliverySubsystem, CoralControlTargetSpeeds.Feed, CoralControlTargetSpeeds.Feed, container.m_liftSubsystem)
            .andThen(new IndexCoralCommand(container.m_coralIndexSubsystem, CoralIndexTargetPositions.Center)));
        NamedCommands.registerCommand("Move Lift to Base", new LiftMoveToLevelCommand(container.m_liftSubsystem, liftTargetLevels.Base));
      //  NamedCommands.registerCommand("Move Lift to L2", new LiftMoveToLevelCommand(container.m_liftSubsystem, liftTargetLevels.L2));
      //  NamedCommands.registerCommand("Move Lift to L3", new LiftMoveToLevelCommand(container.m_liftSubsystem, liftTargetLevels.L3));
        NamedCommands.registerCommand("Move Lift to L4", new LiftMoveToLevelCommand(container.m_liftSubsystem, liftTargetLevels.L4));
        NamedCommands.registerCommand("Intake Coral from Wall", new IntakeCoralCommand(container.m_coralDeliverySubsystem));
        // NamedCommands.registerCommand("Low Reef Algae Removal", new AlgaeReefRemovelSequentialCommand(container.m_liftSubsystem, liftTargetPositions.L2, container.m_driveSubsystem, container.m_aprilTagPID, container.m_algaeClawSubsystem));
        // NamedCommands.registerCommand("High Reef Algae Removal", new AlgaeReefRemovelSequentialCommand(container.m_liftSubsystem, liftTargetPositions.L3, container.m_driveSubsystem, container.m_aprilTagPID, container.m_algaeClawSubsystem));
    }
}
