// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignWithIntakeCommand;
import frc.robot.commands.AlignWithReefCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.PrepareAlgaeIntakeCommand;
import frc.robot.commands.RetractClimbCommand;
import frc.robot.commands.DeliverCoralCommand;
import frc.robot.commands.ExtendClimbCommand;
import frc.robot.commands.IndexCoralCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.L4CoralControlSpeed;
import frc.robot.commands.LiftMoveToLevelCommand;
import frc.robot.commands.ShootAlgaeCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.cwtech.AprilTagPID;
import frc.robot.cwtech.Conditioning;
import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.CoralIndexSubsystem;
import frc.robot.subsystems.CoralIndexSubsystem.CoralIndexTargetPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralControlTargetSpeeds;
import frc.robot.subsystems.LiftSubsystem.liftTargetLevels;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static double kDriveYExponent = 2;
  private static double kDriveXExponent = 2;
  
  public final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final CoralDeliverySubsystem m_coralDeliverySubsystem = new CoralDeliverySubsystem();
  public final AlgaeClawSubsystem m_algaeClawSubsystem = new AlgaeClawSubsystem();
  public final AprilTagPID m_aprilTagPID = new AprilTagPID(m_driveSubsystem);
  public final CoralIndexSubsystem m_coralIndexSubsystem = new CoralIndexSubsystem();
  //public final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  private final SendableChooser<Command> m_autoChooser;

  private final Conditioning m_driveXConditioning = new Conditioning();
  private final Conditioning m_driveYConditioning = new Conditioning();
  private final Conditioning m_turnConditioning = new Conditioning();
  private static double m_speedMultiplier = 0.25;
  private final DoubleSubscriber m_speedSub;

  private final CommandJoystick m_leftJoystick = new CommandJoystick(RobotMap.kLeftJoystick);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(RobotMap.kRightJoystick);
  private final CommandJoystick m_buttonBox = new CommandJoystick(RobotMap.kButtonBox); 
  // private final CommandXboxController m_driverController = new CommandXboxController(RobotMap.kXboxTester);

  private final Robot m_robot;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;
    
    Autos.registerPathPlannerNamedCommands(this);
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser); //Does this need to be updated to not use smart dashboard?

    m_driveXConditioning.setDeadband(0.15);
    m_driveXConditioning.setExponent(kDriveXExponent);
    m_driveYConditioning.setDeadband(0.15);
    m_driveYConditioning.setExponent(kDriveYExponent);
    m_turnConditioning.setDeadband(0.2);
    m_turnConditioning.setExponent(1.4);
    // Configure the trigger bindings
    configureBindings();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Drive");

    var speedSub = datatable.getDoubleTopic("Speed Multiplier");
    speedSub.publish().set(m_speedMultiplier);
    m_speedSub = speedSub.subscribe(m_speedMultiplier);
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
      () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
      () -> m_robot.isTeleopEnabled()));
    m_rightJoystick.button(RobotMap.kRightResetNavXButton).onTrue(new InstantCommand(() -> {m_driveSubsystem.resetNavX();}));
    m_leftJoystick.button(RobotMap.kLeftLockWheels).whileTrue(m_driveSubsystem.lockWheelsCommand());
    m_leftJoystick.button(RobotMap.kLeftL4DeliverButton).whileTrue(new L4CoralControlSpeed(m_coralDeliverySubsystem));
    m_leftJoystick.button(RobotMap.kLeftAlgaeClawRetractButton).onTrue(new InstantCommand(() -> {m_algaeClawSubsystem.retractClawArms();}));
    m_rightJoystick.button(RobotMap.kRightAlignWithReefButton).onTrue(new AlignWithReefCommand(m_driveSubsystem, m_aprilTagPID));
    //m_leftJoystick.button(RobotMap.kLeftExtendClimbButton).whileTrue(new ExtendClimbCommand(m_climbSubsystem));
    //m_leftJoystick.button(RobotMap.kLeftRetractClimbButton).whileTrue(new RetractClimbCommand(m_climbSubsystem));
    //m_rightJoystick.button(RobotMap.kRightPathfindToReefButton).onTrue(m_driveSubsystem.driveToReefPose());
    m_rightJoystick.button(RobotMap.kRightAlignWithIntakeButton).onTrue(new AlignWithIntakeCommand(m_driveSubsystem, m_aprilTagPID));
    m_leftJoystick.button(RobotMap.kLeftDeliverL1Button)
    .onTrue(new DeliverCoralCommand(m_coralDeliverySubsystem.m_deliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.L1)
    .andThen(new IndexCoralCommand(m_coralIndexSubsystem, CoralIndexTargetPositions.Intake)));

    m_buttonBox.button(RobotMap.kIndexCoralLeftButton).onTrue(new IndexCoralCommand(m_coralIndexSubsystem, CoralIndexTargetPositions.Left));
    m_buttonBox.button(RobotMap.kIndexCoralRightButton).onTrue(new IndexCoralCommand(m_coralIndexSubsystem, CoralIndexTargetPositions.Right));
    m_buttonBox.button(RobotMap.kWallIntake).onTrue(new IntakeCoralCommand(m_coralDeliverySubsystem)
       .alongWith(new IndexCoralCommand(m_coralIndexSubsystem, CoralIndexTargetPositions.Intake)));
    m_buttonBox.button(RobotMap.kDeliverCoralButton)
      .onTrue(new DeliverCoralCommand(m_coralDeliverySubsystem.m_deliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.Feed)
      .andThen(new IndexCoralCommand(m_coralIndexSubsystem, CoralIndexTargetPositions.Intake)));

    m_buttonBox.button(RobotMap.kPlaceAtL2Button).onTrue(new LiftMoveToLevelCommand(m_liftSubsystem, liftTargetLevels.L2));
    m_buttonBox.button(RobotMap.kPlaceAtL3Button).onTrue(new LiftMoveToLevelCommand(m_liftSubsystem, liftTargetLevels.L3));
    m_buttonBox.button(RobotMap.kPlaceAtL4Button).onTrue(new LiftMoveToLevelCommand(m_liftSubsystem, liftTargetLevels.L4));
    m_buttonBox.button(RobotMap.kMoveLiftToBase).onTrue(new LiftMoveToLevelCommand(m_liftSubsystem, liftTargetLevels.Base));

    m_buttonBox.button(RobotMap.kAlgaeIntakePrep).onTrue(new PrepareAlgaeIntakeCommand(m_liftSubsystem, m_algaeClawSubsystem));
     m_buttonBox.button(RobotMap.kAlgaeIntakeLow).onTrue(new LiftMoveToLevelCommand(m_liftSubsystem, liftTargetLevels.LowAlage));
     m_buttonBox.button(RobotMap.kAlgaeIntakeHigh).onTrue(new LiftMoveToLevelCommand(m_liftSubsystem, liftTargetLevels.HighAlage));
     m_buttonBox.button(RobotMap.kDeliverAlgaeButton).onTrue(new ShootAlgaeCommand(m_algaeClawSubsystem)
      .andThen(new InstantCommand(() -> {m_algaeClawSubsystem.retractClawArms();})));
   //    .onTrue(new LiftMoveToLevelCommand(m_liftSubsystem, liftTargetLevels.Processor)
   //    .andThen(new ShootAlgaeCommand(m_algaeClawSubsystem)));
    

    // m_buttonBox.button(RobotMap.kplaceAtL3Button).onTrue(new LiftDriveToPositionCommand(m_liftSubsystem, liftTargetPositions.L3)
    //   .andThen(new DeliverCoralCommand(m_coralDeliverySubsystem.DeliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.Feed, CoralControlTargetSpeeds.Feed)));
    
    // m_buttonBox.button(RobotMap.kplaceAtL2Button).onTrue(new LiftDriveToPositionCommand(m_liftSubsystem, liftTargetPositions.L2)
    //   .andThen(new DeliverCoralCommand(m_coralDeliverySubsystem.DeliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.Feed, CoralControlTargetSpeeds.Feed)));

    // m_coralDeliverySubsystem.setDefaultCommand(
    //   new CoralIndexDirectDriveCommand(m_coralDeliverySubsystem, () -> m_driverController.getLeftY()));
    // m_liftSubsystem.setDefaultCommand(
    //   new LiftDirectDriveCommand(m_liftSubsystem, () -> m_driverController.getLeftY()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void initialize()
  {
    m_speedMultiplier = m_speedSub.get();

    m_driveSubsystem.initialize();
    m_liftSubsystem.initialize();
  }

  public double getDriveXInput()
  {
    // We getY() here because of the FRC coordinate system being turned 90 degrees
    return m_driveXConditioning.condition(-m_leftJoystick.getY())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getDriveYInput()
  {
    // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
    return m_driveYConditioning.condition(-m_leftJoystick.getX())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getTurnInput()
  {
    return m_turnConditioning.condition(-m_rightJoystick.getX())
            * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond
            * m_speedMultiplier;
  }

  public void updateDashboard()
  {
    m_aprilTagPID.updateAprilTagSmartDashboard();
  }
}
