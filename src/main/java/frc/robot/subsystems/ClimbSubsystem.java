// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  private TalonFX m_climbMotor = new TalonFX(RobotMap.kClimbMotor);
  private static final int kClimbCurrentLimitAmps = 100;    //taken from SwerveModuleThrifty

  private double m_climbVelocity = 0.5;

  private DoublePublisher m_positionPub;
  private DoublePublisher m_appliedOutputPub;
  private DoubleSubscriber m_targetVelocitySub;
  private BooleanSubscriber m_goToTargetVelocitySub;
  private BooleanPublisher m_goToTargetVelocityPub;

  public ClimbSubsystem() {

  var krakenLimitsConfig =  new CurrentLimitsConfigs()
    .withStatorCurrentLimit(kClimbCurrentLimitAmps)
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimitEnable(true);
  m_climbMotor.getConfigurator().apply(krakenLimitsConfig);

  m_climbMotor.setNeutralMode(NeutralModeValue.Brake);

  final String name = "Climb Subsystem";
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable datatable = inst.getTable(name);

  m_positionPub = datatable.getDoubleTopic("Position").publish();
  m_appliedOutputPub = datatable.getDoubleTopic("Applied Output").publish();

  var targetVelocityTopic = datatable.getDoubleTopic("Target Velocity");
  targetVelocityTopic.publish().set(0.0);
  m_targetVelocitySub = targetVelocityTopic.subscribe(0.0);

  var goToTargetVelocityTopic = datatable.getBooleanTopic("update Target Velocity");
  m_goToTargetVelocityPub = goToTargetVelocityTopic.publish();
  m_goToTargetVelocityPub.set(false);
  m_goToTargetVelocitySub = goToTargetVelocityTopic.subscribe(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //updateDashboard();
  }

  private void updateDashboard()
  {
    m_positionPub.set(getClimbPosition());
    m_appliedOutputPub.set(m_climbMotor.getDutyCycle().getValueAsDouble());

    if(m_goToTargetVelocitySub.get())
    {
      m_climbVelocity = m_targetVelocitySub.get();
      m_goToTargetVelocityPub.set(false);
    }
  }

  public double getClimbPosition ()
  {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  public void retractClimb()
  {
    m_climbMotor.set(m_climbVelocity);
  }

  public void extendClimb()
  {
    m_climbMotor.set(-m_climbVelocity);
  }

  public void stopClimb()
  {
    m_climbMotor.set(0.0);
  }
}
