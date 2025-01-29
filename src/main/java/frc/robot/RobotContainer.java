// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LiftDirectDriveCommand;
import frc.robot.commands.LiftDriveToPositionCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.cwtech.Conditioning;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.liftTargetPositions;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
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
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  Conditioning m_driveXConditioning = new Conditioning();
  Conditioning m_driveYConditioning = new Conditioning();
  Conditioning m_turnConditioning = new Conditioning();
  double m_speedMultiplier = 0.85;

  Joystick m_leftJoystick = new Joystick(RobotMap.kLeftJoystick);
  Joystick m_rightJoystick = new Joystick(RobotMap.kRightJoystick);
  //Joystick m_buttonBox = new Joystick(RobotMap.kButtonBox);

  //JoystickButton m_goToL4Button =new JoystickButton(m_buttonBox, RobotMap.kgoToL4Button);


  Robot m_robot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
      () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
      () -> m_robot.isTeleopEnabled()));

    m_liftSubsystem.setDefaultCommand(
      new LiftDirectDriveCommand(m_liftSubsystem, () -> m_driverController.getLeftY()));

    //m_goToL4Button.onTrue(new LiftDriveToPositionCommand(m_liftSubsystem, liftTargetPositions.L4));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
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
