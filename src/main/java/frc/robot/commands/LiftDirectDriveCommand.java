// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class LiftDirectDriveCommand extends Command {
    private LiftSubsystem m_liftsubsystem;
    private Supplier<Double> m_yJoystickSupplier;

    public LiftDirectDriveCommand(
        LiftSubsystem liftSubsystem,
        Supplier<Double> yJoystick)
    {
        m_liftsubsystem = liftSubsystem;

        m_yJoystickSupplier = yJoystick;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute()
    {
        double power = m_yJoystickSupplier.get();
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
        
        power = power * power * power;
        m_liftsubsystem.directDrive(power);
    }

    @Override
    public void end(boolean interrupted) {
        m_liftsubsystem.directDrive(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
