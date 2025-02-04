// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new AlgaeClawSubsystem. */
  private final SparkMax m_clawShootSparkMax = new SparkMax(RobotMap.kAlgaeClawShootMotor, MotorType.kBrushless);
  private final SparkMax m_clawIntakeSparkMax = new SparkMax(RobotMap.kAlgaeClawIntakeMotor, MotorType.kBrushless);
  private SparkRelativeEncoder m_clawShootEncoder;
  private final SparkMaxConfig m_clawShootConfig;
  private final SparkMaxConfig m_clawIntakeConfig;
  private Solenoid m_solenoid;

  private double m_lastTargetVelocity = 0;
  private double m_clawIntakeSpeed = 0.2; //random number
  private double m_clawShootSpeed = 0.2; //random number

  private final DoubleSubscriber m_clawIntakeSpeedSub;
  private final DoubleSubscriber m_clawShootSpeedSub;

  public AlgaeClawSubsystem() {
    final String name = "Algae Claw Subsystem";

    m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kAlgaeClawSolenoid); //Random Channel
    m_clawShootConfig = new SparkMaxConfig();
    m_clawShootConfig.idleMode(IdleMode.kBrake);

    m_clawIntakeConfig = new SparkMaxConfig();
    m_clawIntakeConfig.idleMode(IdleMode.kBrake);

    m_clawShootSparkMax.configure(m_clawShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_clawIntakeSparkMax.configure(m_clawIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_clawShootEncoder = (SparkRelativeEncoder) m_clawShootSparkMax.getEncoder(); 

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("AlgaeClawSubsystem");

    var clawIntakeSpeedSub = datatable.getDoubleTopic(name + "/ClawIntakeSpeed");
    clawIntakeSpeedSub.publish().set(0);
    m_clawIntakeSpeedSub = clawIntakeSpeedSub.subscribe(0.0);

    var clawShootSpeedSub = datatable.getDoubleTopic(name + "/ClawShootSpeed");
    clawShootSpeedSub.publish().set(0);
    m_clawShootSpeedSub = clawShootSpeedSub.subscribe(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dashboardUpdate();
  }

  private void dashboardUpdate ()
  {
    m_clawIntakeSpeed = m_clawIntakeSpeedSub.get();
    m_clawShootSpeed = m_clawShootSpeedSub.get();
  }

  public void intakeAlgae ()
  {
    m_clawShootSparkMax.set(-m_clawIntakeSpeed);
    m_clawIntakeSparkMax.set(-m_clawIntakeSpeed);
  }

  public boolean isAtTargetShootVelocity ()
  {
    return Math.abs(m_lastTargetVelocity - m_clawShootEncoder.getPosition()) < 10;
  }
  
  public void feedAlgaeOut ()
  {
    m_clawIntakeSparkMax.set(m_clawIntakeSpeed);
  }

  public void feedAlgaeIntoProcessor ()
  {
    feedAlgaeOut();
    m_clawShootSparkMax.set(m_clawShootSpeed);
  }

  public void extendSolenoid ()
  {
    m_solenoid.set(true);
  }

  public void retractSolenoid ()
  {
    m_solenoid.set(false);
  }

  public void stopClawShootMotor ()
  {
    m_clawShootSparkMax.set(0);
  }

  public void stopClawIntakeMotor ()
  {
    m_clawShootSparkMax.set(0);
  }
}
