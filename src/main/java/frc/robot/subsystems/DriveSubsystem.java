// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveModuleThrifty m_frontLeft;
    private final SwerveModuleThrifty m_frontRight;
    private final SwerveModuleThrifty m_backLeft;
    private final SwerveModuleThrifty m_backRight;
    private final AHRS m_navX;

    private boolean m_initalized = false;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// kMaxSpeedMetersPerSecond /
                                                                          // Math.hypot(0.381, 0.381);
    private static final double kHalfTrackWidthMeters = 0.5715 / 2.0;
    private final Translation2d kFrontLeftLocation = new Translation2d(kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kFrontRightLocation = new Translation2d(kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
    private final Translation2d kBackLeftLocation = new Translation2d(-kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kBackRightLocation = new Translation2d(-kHalfTrackWidthMeters, -kHalfTrackWidthMeters);

    private int m_ticks = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem()
    {
        m_navX = new AHRS(NavXComType.kMXP_SPI);
        m_kinematics = new SwerveDriveKinematics(kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation,
                kBackRightLocation);

        m_frontLeft = new SwerveModuleThrifty(RobotMap.kFrontLeftModule,
                RobotMap.kZeroedFrontLeft, "Front Left");
        m_frontRight = new SwerveModuleThrifty(RobotMap.kFrontRightModule,
                RobotMap.kZeroedFrontRight, "Front Right");
        m_backLeft = new SwerveModuleThrifty(RobotMap.kBackLeftModule,
                RobotMap.kZeroedBackLeft, "Back Left");
        m_backRight = new SwerveModuleThrifty(RobotMap.kBackRigvhtModule,
                RobotMap.kZeroedBackRight, "Back Right");

        m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle(), getPositions());

        // need to add path planner stuff!
    }

    public void initalize() {
        if (m_initalized)
            return;
        resetSteeringMotorsToAbsolute();
        m_navX.reset();
        m_initalized = true;
    }

    private void resetSteeringMotorsToAbsolute()
    {
        m_frontLeft.resetAngleEncoderToAbsolute();
        m_frontRight.resetAngleEncoderToAbsolute();
        m_backLeft.resetAngleEncoderToAbsolute();
        m_backRight.resetAngleEncoderToAbsolute();
    }

    @Override
    public void periodic() {
        m_odometry.update(getAngle(), getPositions());

        m_ticks++;
        if (m_ticks % 15 != 7)
            return;

        SmartDashboard.putNumber(getName() + "/Angle", getAngle().getDegrees());
        // SmartDashboard.putNumber(getName() + "/Roll", m_navX.getRoll());
        // SmartDashboard.putNumber(getName() + "/Pitch", m_navX.getPitch());
        
        // var botPose = LimelightHelpers.getBotPose2d("limelight-swtech");
        // SmartDashboard.putNumber(getName() + "/April Tag ID", botpose);
        // SmartDashboard.putNumber(getName() + "/Distance X", botpose.getX());
        // SmartDashboard.putNumber(getName() + "/Distance Y", botpose.getY());
        // SmartDashboard.putNumber(getName() + "/Distance Z", botpose.getZ());
    }

    public void dashboardUpdate() {
        m_frontLeft.dashboardUpdate();
        m_frontRight.dashboardUpdate();
        m_backLeft.dashboardUpdate();
        m_backRight.dashboardUpdate();
    }

    private void driveBotRelative(ChassisSpeeds chassisSpeeds)
    {
        drive(m_kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    private void drive(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public void drive(double xMetersPerSecond, double yMetersPerSecond,
            double rotationRadiansPerSecond, boolean fieldRelative)
    {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond,
                        getAngle())
                : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond));

        drive(states);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public ChassisSpeeds getChassisSpeeds()
    {
        return m_kinematics.toChassisSpeeds(
            m_frontLeft.getSwerveModuleState(),
            m_frontRight.getSwerveModuleState(),
            m_backLeft.getSwerveModuleState(),
            m_backRight.getSwerveModuleState());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setDesiredState(SwerveModuleState[] states) {
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public Rotation2d getAngle() {
        return m_navX.getRotation2d();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getAngle(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] swerveStates = { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_backLeft.getPosition(), m_backRight.getPosition() };
        return swerveStates;
    }

    public void stopAndLockWheels() {
        m_frontLeft.lockWheelAtAngleInDegrees(45);
        m_frontRight.lockWheelAtAngleInDegrees(-45);
        m_backLeft.lockWheelAtAngleInDegrees(-45);
        m_backRight.lockWheelAtAngleInDegrees(45);
    }

    public void checkRelativeEncoderToAbsoluteEncoder()
    {
        m_frontLeft.checkRelativeEncoderToAbsoluteEncoder();
        m_backLeft.checkRelativeEncoderToAbsoluteEncoder();
        m_frontRight.checkRelativeEncoderToAbsoluteEncoder();
        m_backRight.checkRelativeEncoderToAbsoluteEncoder();
    }

    public void resetNavX() {
        m_navX.reset();
        setStartAngle(0);
        resetSteeringMotorsToAbsolute();
    }

    public Pose2d m_targetDriveToPose2d;
    public void resetPoseFromLimelight()
    {
        // resetOdometry(new Pose2d( ));   // from lime light

        // switch based on april tag # for target position
        int targetAprilTag = 8;
        switch (targetAprilTag) {
            case 8:
                m_targetDriveToPose2d = new Pose2d(1.5, 5.52, Rotation2d.fromDegrees(0));
                break;
        
            default:
                break;
        }
    }

    public void setStartAngle(double angle) {
        m_navX.setAngleAdjustment(angle);
    }
}
