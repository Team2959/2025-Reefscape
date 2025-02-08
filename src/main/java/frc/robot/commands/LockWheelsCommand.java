package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LockWheelsCommand extends Command {
    DriveSubsystem m_DriveSubsystem;

    public LockWheelsCommand(DriveSubsystem driveSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.stopAndLockWheels();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

