// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends Command {
    private DriveSubsystem m_driveSubsystem;
    private Supplier<Double> m_xJoystickSupplier;
    private Supplier<Double> m_yJoystickSupplier;
    private Supplier<Double> m_turnJoystickSupplier;
    private Supplier<Boolean> m_isTeleopEnabled;

    public TeleOpDriveCommand(DriveSubsystem driveSubsystem,
            Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> turnJoystick,
            Supplier<Boolean> isTeleopEnabled) {
        m_driveSubsystem = driveSubsystem;

        m_xJoystickSupplier = xJoystick;
        m_yJoystickSupplier = yJoystick;
        m_turnJoystickSupplier = turnJoystick;
        m_isTeleopEnabled = isTeleopEnabled;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_isTeleopEnabled.get())
            m_driveSubsystem.drive(m_xJoystickSupplier.get(), m_yJoystickSupplier.get(), m_turnJoystickSupplier.get(),
                    false);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
