// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cwtech;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;

/** Add your docs here. */
public class AprilTagPID
{
    private DriveSubsystem m_driveSubsystem;
    private double kSpeedKp = 0.75;
    private double kRotationKp = 0.035;
    private double kSpeedKi = 0;
    private double kRotationKi = 0;
    private double kSpeedKd = 0;
    private double kRotationKd = 0;
    private double m_targetXPosition = 0;
    private double m_targetZPosition = 0;
    private double m_targetRotationPosition = 0;
    private double m_deltaRotation = 0;
    private double m_deltaX = 0;
    private double m_deltaZ = 0;

    PIDController m_rotationController = new PIDController(kRotationKp, kRotationKi, kRotationKd);
    PIDController m_xSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);
    PIDController m_zSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);

    public AprilTagPID(DriveSubsystem driveSubsystem)
    {
        m_driveSubsystem = driveSubsystem;

        SmartDashboard.putNumber("AprilTag/kP Speed", kSpeedKp);
        SmartDashboard.putNumber("AprilTag/kP Rotation", kRotationKp);
        SmartDashboard.putNumber("AprilTag/kI Speed", kSpeedKi);
        SmartDashboard.putNumber("AprilTag/kI Rotation", kRotationKi);
        SmartDashboard.putNumber("AprilTag/kD Speed", kSpeedKd);
        SmartDashboard.putNumber("AprilTag/kD Rotation", kRotationKd);
        SmartDashboard.putBoolean("AprilTag/Update PIDs", false);
        
    }

    public void updateAprilTagSmartDashboard()
    {
        boolean updatePID = SmartDashboard.getBoolean("AprilTag/Update PIDs", false);
        if ( updatePID == true)
         {
             updatePID();
            SmartDashboard.putBoolean("AprilTag/Update PIDs", false);
         }
    }   

    private void updatePID()
    {
        kSpeedKp = SmartDashboard.getNumber("AprilTag/kP Speed", kSpeedKp);
        kRotationKp = SmartDashboard.getNumber("AprilTag/kP Rotation", kRotationKp);
        kSpeedKi = SmartDashboard.getNumber("AprilTag/kI Speed", kSpeedKi);
        kRotationKi = SmartDashboard.getNumber("AprilTag/kI Rotation", kRotationKi);
        kSpeedKd = SmartDashboard.getNumber("AprilTag/kD Speed", kSpeedKd);
        kRotationKd = SmartDashboard.getNumber("AprilTag/kD Rotation", kRotationKd);
    }

    public void setTargetPosition(double xPosition, double zPosition, double rotation)
    {
        m_xSpeedController.setSetpoint(xPosition);
        m_zSpeedController.setSetpoint(zPosition);
        m_rotationController.setSetpoint(rotation);
        m_targetXPosition = xPosition;
        m_targetZPosition = zPosition;
        m_targetRotationPosition = rotation;
    }

    private double zSpeed()
    {
        double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        double tz = robotSpace[2];
        double targetingForwardSpeed = m_zSpeedController.calculate(tz);
        targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
        m_deltaZ = Math.abs(m_targetZPosition - tz);
        return -targetingForwardSpeed;
    }

    private double ySpeed()
    {
        double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        double tx = robotSpace[0];
        double targetingForwardSpeed = m_xSpeedController.calculate(tx);
        targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
        targetingForwardSpeed *= -1.0;
        m_deltaX = Math.abs(m_targetXPosition - tx);
        return targetingForwardSpeed;
    }

    private double rotationTarget()
    {
        double botAngle = m_driveSubsystem.getAngle().getDegrees();
        double remappedAngle = remapAngle(botAngle);
        double targetingAngularVelocity = m_rotationController.calculate(remappedAngle);
        targetingAngularVelocity *= DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
        m_deltaRotation = Math.abs(m_targetRotationPosition - remappedAngle);
        return targetingAngularVelocity;
    }

    private double remapAngle(double fromNavX)
    {
        double remapAngle = fromNavX % 360;
        if (remapAngle <= -45) 
        {
            remapAngle = remapAngle + 360;
        }
        return remapAngle;
    }

    public void driveToTarget()
    {
        m_driveSubsystem.drive(-zSpeed(), ySpeed(), rotationTarget(), false);
    }

    public void driveToTargetNoZ()
    {
        m_driveSubsystem.drive(0, ySpeed(), rotationTarget(), false);
    }

    public boolean atTargetPosition()
    {
       return m_deltaRotation < 0.5 && m_deltaX < 0.05 && m_deltaZ < 0.05;
    }
}
