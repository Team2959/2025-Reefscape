// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class CoralIndexDirectDriveCommand extends Command {
    private CoralDeliverySubsystem m_coralSubsystem;
    private Supplier<Double> m_yJoystickSupplier;

    public CoralIndexDirectDriveCommand(
        CoralDeliverySubsystem coralSubsystem,
        Supplier<Double> yJoystick)
    {
        m_coralSubsystem = coralSubsystem;

        m_yJoystickSupplier = yJoystick;

        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute()
    {
        double power = -m_yJoystickSupplier.get();
        if (Math.abs(power) < 0.1)
        {
            power = 0;
        }
        else if ( power < 0)
        {
            power += 0.1;
        }
        else
        {
            power -= 0.1;
        }
        
        m_coralSubsystem.directDrive(power);
    }

    @Override
    public void end(boolean interrupted) {
        m_coralSubsystem.directDrive(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
