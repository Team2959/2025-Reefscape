// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignWithReefCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralIndexDirectDriveCommand;
import frc.robot.commands.DeliverCoralCommand;
import frc.robot.commands.IndexCoralCommand;
import frc.robot.commands.IntakeAlgaeCommand;
import frc.robot.commands.LiftDirectDriveCommand;
import frc.robot.commands.LiftDriveToPositionCommand;
import frc.robot.commands.LockWheelsCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.cwtech.AprilTagPID;
import frc.robot.cwtech.Conditioning;
import frc.robot.subsystems.AlgaeClawSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralControlTargetSpeeds;
import frc.robot.subsystems.CoralDeliverySubsystem.CoralIndexTargetPositions;
import frc.robot.subsystems.LiftSubsystem.liftTargetPositions;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  private final SendableChooser<Command> m_autoChooser;

  private final Conditioning m_driveXConditioning = new Conditioning();
  private final Conditioning m_driveYConditioning = new Conditioning();
  private final Conditioning m_turnConditioning = new Conditioning();
  private static double m_speedMultiplier = 1.0;

  private final Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
  private final Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
  // private final Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox); 
  // private final CommandXboxController m_driverController = new CommandXboxController(RobotMap.kXboxTester);

  // Driver Buttons
  private final JoystickButton m_resetNavX = new JoystickButton(m_rightJoystick, RobotMap.kRightResetNavXButton);
  private final JoystickButton m_lockWheeButton = new JoystickButton(m_leftJoystick, RobotMap.kLeftLockWheels);
  private final JoystickButton m_leftTroughPlaceButton = new JoystickButton(m_leftJoystick, RobotMap.kLeftTroughButton);
  private final JoystickButton m_rightTroughPlaceButton = new JoystickButton(m_rightJoystick, RobotMap.kRightTroughButton);

  // Co-Pilot buttons
  // private final JoystickButton m_placeAtL4Button = new JoystickButton(m_buttonBox, RobotMap.kplaceAtL4Button);
  // private final JoystickButton m_placeAtL3Button = new JoystickButton(m_buttonBox, RobotMap.kplaceAtL3Button);
  // private final JoystickButton m_placeAtL2Button = new JoystickButton(m_buttonBox, RobotMap.kplaceAtL2Button);
  // private final JoystickButton m_intakeAlgaeButton = new JoystickButton(m_buttonBox, RobotMap.kintakeAlgaeButton);

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
    m_resetNavX.onTrue(new InstantCommand(() -> {m_driveSubsystem.resetNavX();}));
    m_lockWheeButton.whileTrue(new LockWheelsCommand(m_driveSubsystem));
    m_leftTroughPlaceButton.onTrue(new DeliverCoralCommand(CoralDeliverySubsystem.DeliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.L1SlowSpeed, CoralControlTargetSpeeds.L1FastSpeed));
    m_rightTroughPlaceButton.onTrue(new DeliverCoralCommand(CoralDeliverySubsystem.DeliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.L1FastSpeed, CoralControlTargetSpeeds.L1SlowSpeed));

    // m_coralDeliverySubsystem.setDefaultCommand(
    //   new CoralIndexDirectDriveCommand(m_coralDeliverySubsystem, () -> m_driverController.getLeftY()));
    // m_liftSubsystem.setDefaultCommand(
    //   new LiftDirectDriveCommand(m_liftSubsystem, () -> m_driverController.getLeftY()));

    // m_placeAtL4Button.onTrue(new AlignWithReefCommand(m_driveSubsystem, m_aprilTagPID)
    //  .andThen(new LiftDriveToPositionCommand(m_liftSubsystem, liftTargetPositions.L4)
    //  .andThen(new DeliverCoralCommand(CoralDeliverySubsystem.DeliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.Feed, CoralControlTargetSpeeds.Feed)
    //   .andThen(new LiftDriveToPositionCommand(m_liftSubsystem, liftTargetPositions.Base)
    //   .andThen(new IndexCoralCommand(m_coralDeliverySubsystem, CoralIndexTargetPositions.Center))))));

    // m_placeAtL3Button.onTrue(new LiftDriveToPositionCommand(m_liftSubsystem, liftTargetPositions.L3)
    //   .andThen(new DeliverCoralCommand(CoralDeliverySubsystem.DeliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.Feed, CoralControlTargetSpeeds.Feed)));
    
    // m_placeAtL2Button.onTrue(new LiftDriveToPositionCommand(m_liftSubsystem, liftTargetPositions.L2)
    //   .andThen(new DeliverCoralCommand(CoralDeliverySubsystem.DeliveryWaitSeconds, m_coralDeliverySubsystem, CoralControlTargetSpeeds.Feed, CoralControlTargetSpeeds.Feed)));

    // m_intakeAlgaeButton.whileTrue(new IntakeAlgaeCommand(m_algaeClawSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
    // return m_autoChooser.getSelected();
  }

  public void driveInitialize()
  {
    m_driveSubsystem.initalize();
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
}
