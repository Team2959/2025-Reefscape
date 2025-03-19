// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.cwtech.AprilTagHelper;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModuleThrifty m_frontLeft;
    private final SwerveModuleThrifty m_frontRight;
    private final SwerveModuleThrifty m_backLeft;
    private final SwerveModuleThrifty m_backRight;
    private final AHRS m_navX;
    public final SwerveDrivePoseEstimator m_poseEstimator;

    private boolean m_initalized = false;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;
    
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;

    public static final double kMaxSpeedMetersPerSecond = 5.836; //Based off of 6000 rpm for Kraken x60
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// kMaxSpeedMetersPerSecond /
                                                                          // Math.hypot(0.381, 0.381);
    // 2025 center to center on wheels is 24.5" -> 0.6223 meters
    private static final double kHalfTrackWidthMeters = 0.6223 / 2.0;
    private final Translation2d kFrontLeftLocation = new Translation2d(kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kFrontRightLocation = new Translation2d(kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
    private final Translation2d kBackLeftLocation = new Translation2d(-kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kBackRightLocation = new Translation2d(-kHalfTrackWidthMeters, -kHalfTrackWidthMeters);

    private int m_ticks = 0;

    private double k1XCoordinate = 5.812;
    private double k1YCoordinate = 4.025;
    private double k2XCoordinate = 5.139;
    private double k2YCoordinate = 2.881;
    private double k3XCoordinate = 3.830;
    private double k3YCoordinate = 2.881;
    private double k4XCoordinate = 3.177;
    private double k4YCoordinate = 4.025;
    private double k5XCoordinate = 3.830;
    private double k5YCoordinate = 5.166;
    private double k6XCoordinate = 5.139;
    private double k6YCoordinate = 5.166;

    private DoublePublisher m_mt2XCoordinatePub;
    private DoublePublisher m_mt2YCoordinatePub;
    private DoublePublisher m_mt2RotationPub;
    private IntegerPublisher m_tidPub;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem()
    {
        m_navX = new AHRS(NavXComType.kUSB1);
        m_kinematics = new SwerveDriveKinematics(kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation,
                kBackRightLocation);

        m_frontLeft = new SwerveModuleThrifty(RobotMap.kFrontLeftModule,
                RobotMap.kZeroedFrontLeft, RobotMap.kFrontLeftAnalogInput, "Front Left");
        m_frontRight = new SwerveModuleThrifty(RobotMap.kFrontRightModule,
                RobotMap.kZeroedFrontRight, RobotMap.kFrontRightAnalogInput, "Front Right");
        m_backLeft = new SwerveModuleThrifty(RobotMap.kBackLeftModule,
                RobotMap.kZeroedBackLeft, RobotMap.kBackLeftAnalogInput,  "Back Left");
        m_backRight = new SwerveModuleThrifty(RobotMap.kBackRigvhtModule,
                RobotMap.kZeroedBackRight, RobotMap.kBackRightAnalogInput, "Back Right");

        m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle(), getPositions());
    
     /* From limelight example project on github: https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java#L35
         Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
        below are robot specific, and should be tuned. */
        m_poseEstimator =
        new SwerveDrivePoseEstimator(
            m_kinematics,
            m_navX.getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, 5 * kDegreesToRadians), 
            VecBuilder.fill(0.5, 0.5, 30 * kDegreesToRadians)); //these numbers need to be tuned


    try{
        var config = RobotConfig.fromGUISettings();

        //   Configure AutoBuilder last
        AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveBotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants fromm Pathplanner site
                      new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants from Pathplanner site
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
          );
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }

        final String name = "Drive Subsystem";
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable datatable = inst.getTable(name);

        m_mt2XCoordinatePub = datatable.getDoubleTopic("mt2 X-Coordinate").publish();
        m_mt2YCoordinatePub = datatable.getDoubleTopic("mt2 Y-Coordinate").publish();
        m_mt2RotationPub = datatable.getDoubleTopic("mt2 Rotation Degrees").publish();
        m_tidPub = datatable.getIntegerTopic("tid").publish();
    }

    public void initialize() {
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
        updateOdemetry();

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
        dashboardUpdate();
    }

    public void dashboardUpdate() {
        m_frontLeft.dashboardUpdate();
        m_frontRight.dashboardUpdate();
        m_backLeft.dashboardUpdate();
        m_backRight.dashboardUpdate();

        var pose2d = m_poseEstimator.getEstimatedPosition();
        m_mt2XCoordinatePub.set(pose2d.getX());
        m_mt2YCoordinatePub.set(pose2d.getY());
        m_mt2RotationPub.set(pose2d.getRotation().getDegrees());

        m_tidPub.set(AprilTagHelper.tidFromLimelight());
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

    private void updateOdemetry()
    {
        // for bot only odometry
        m_odometry.update(getAngle(), getPositions());
      
        // for bot odometry corrected by MegaTag2
        // m_poseEstimator.update(getAngle(),getPositions()); 

        // if(Math.abs(m_navX.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        // {
        //   return;
        // }

        // LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // if (mt2 == null || mt2.tagCount <= 0)
        // {
        //   return;
        // }

        // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        // m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
        // return m_poseEstimator.getEstimatedPosition();
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
        // m_poseEstimator.resetPosition(getAngle(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] swerveStates = { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_backLeft.getPosition(), m_backRight.getPosition() };
        return swerveStates;
    }

    public void stopAndLockWheels() {
        m_frontLeft.lockWheelAtAngleInDegrees(-45);
        m_frontRight.lockWheelAtAngleInDegrees(45);
        m_backLeft.lockWheelAtAngleInDegrees(45);
        m_backRight.lockWheelAtAngleInDegrees(-45);
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

    public Command lockWheelsCommand()
    {
        return this.startEnd(() -> this.stopAndLockWheels(), () -> {});
    }

    public double getReefFieldYCoordinate (int tid)
    {
        switch (tid)
        {
            case 21:
            case 10:
                return k1YCoordinate;
            case 22:
            case 11:
                return k2YCoordinate;
            case 17:
            case 6:
                return k3YCoordinate;
            case 18:
            case 7:
                return k4YCoordinate;
            case 19: 
            case 8:
                return k5YCoordinate;
            case 20:
            case 9:
                return k6YCoordinate;
            default:
                return 0.0;
        }
    }

    public double getReefFieldXCoordinate (int tid)
    {
        switch (tid)
        {
            case 21:
            case 10:
                return k1XCoordinate;
            case 22:
            case 11:
                return k2XCoordinate;
            case 17:
            case 6:
                return k3XCoordinate;
            case 18:
            case 7:
                return k4XCoordinate;
            case 19: 
            case 8:
                return k5XCoordinate;
            case 20:
            case 9:
                return k6XCoordinate;
            default:
                return 0.0;
        }
    }

    public Command driveToReefPose()
    {
        var tid = AprilTagHelper.tidFromLimelight();
        var targetAngle = AprilTagHelper.reefAngleFromTid(tid);
        if (targetAngle < 0 )
        {
            return new InstantCommand();
        }

        var targetPose = new Pose2d(getReefFieldXCoordinate(tid), getReefFieldYCoordinate(tid), Rotation2d.fromDegrees(targetAngle));
        var constraints = new PathConstraints(3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }
}
