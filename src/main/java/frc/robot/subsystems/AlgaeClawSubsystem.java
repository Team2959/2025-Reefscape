// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new AlgaeClawSubsystem. */
  private final SparkMax m_clawShootSparkMax = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax m_clawIntakeSparkMax = new SparkMax(8, MotorType.kBrushless);
  private SparkRelativeEncoder m_clawShootEncoder;
  private SparkRelativeEncoder m_clawIntakeEncoder;
  private final SparkMaxConfig m_clawShootConfig;
  private final SparkMaxConfig m_clawIntakeConfig;
  private SparkClosedLoopController m_clawShootController;
  private Solenoid m_solenoid;


  private final double kClawP = 0;
  private final double kClawI = 0;
  private final double kClawD = 0;
  private double m_lastTargetVelocity = 0;
  private double m_clawIntakeSpeed = 0.2; //random number

  private final DoublePublisher m_clawShootVelocityPub;
  private final DoubleSubscriber m_clawIntakeSpeedSub;
  private final DoubleSubscriber m_clawTargetShootVelocitySub;
  private final DoubleSubscriber m_clawkPSub;
  private final DoubleSubscriber m_clawkISub;
  private final DoubleSubscriber m_clawkDSub;
  private final BooleanPublisher m_updateClawPIDPub;
  private final BooleanSubscriber m_updateClawPIDSub;
  private final BooleanPublisher m_goToTargetCLawVelocityPub;
  private final BooleanSubscriber m_goToTargetCLawVelocitySub;

  public AlgaeClawSubsystem() {
    final String name = "Algae Claw Subsystem";

    m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0); //Random Channel
    m_clawShootConfig = new SparkMaxConfig();
    m_clawShootConfig.idleMode(IdleMode.kBrake);
    m_clawShootConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kClawP, kClawI, kClawD);

    m_clawIntakeConfig = new SparkMaxConfig();
    m_clawIntakeConfig.idleMode(IdleMode.kBrake);

    m_clawShootSparkMax.configure(m_clawShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_clawIntakeSparkMax.configure(m_clawIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_clawShootEncoder = (SparkRelativeEncoder) m_clawShootSparkMax.getEncoder(); 
    m_clawIntakeEncoder = (SparkRelativeEncoder) m_clawIntakeSparkMax.getEncoder(); 
    m_clawShootController = m_clawShootSparkMax.getClosedLoopController();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("AlgaeClawSubsystem");

    m_clawShootVelocityPub = datatable.getDoubleTopic(name + "/ShootVelocity").publish();

    var clawIntakeSpeedSub = datatable.getDoubleTopic(name + "/ClawIntakeSpeed");
    clawIntakeSpeedSub.publish().set(0);
    m_clawIntakeSpeedSub = clawIntakeSpeedSub.subscribe(0.0);

    var clawTargetShootVelocitySub = datatable.getDoubleTopic(name + "/ClawTargetShootVelocity");
    clawTargetShootVelocitySub.publish().set(0);
    m_clawTargetShootVelocitySub = clawTargetShootVelocitySub.subscribe(0.0);

    var clawPSub = datatable.getDoubleTopic(name + "/ShootP");
    clawPSub.publish().set(0);
    m_clawkPSub = clawPSub.subscribe(0.0);

    var clawISub = datatable.getDoubleTopic(name + "/ShootI");
    clawISub.publish().set(0);
    m_clawkISub = clawISub.subscribe(0.0);
    
    var clawDSub = datatable.getDoubleTopic(name + "/ShootD");
    clawDSub.publish().set(0);
    m_clawkDSub = clawDSub.subscribe(0.0);

    var goToTargetShootVelocity = datatable.getBooleanTopic(name + "/goToTargetShootVelocity");
    m_goToTargetCLawVelocityPub = goToTargetShootVelocity.publish();
    m_goToTargetCLawVelocityPub.set(false);
    m_goToTargetCLawVelocitySub = goToTargetShootVelocity.subscribe(false);

    var updateClawShootPID = datatable.getBooleanTopic(name + "/goToTargetShootVelocity");
    m_updateClawPIDPub = updateClawShootPID.publish();
    m_updateClawPIDPub.set(false);
    m_updateClawPIDSub = updateClawShootPID.subscribe(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dashboardUpdate();
  }

  public void dashboardUpdate ()
  {
    m_clawShootVelocityPub.set(m_clawShootEncoder.getVelocity());
    double targetVelocity = m_clawTargetShootVelocitySub.get();
    m_clawIntakeSpeed = m_clawIntakeSpeedSub.get();

    if(m_goToTargetCLawVelocitySub.get())
    {
      m_clawShootController.setReference(targetVelocity, SparkMax.ControlType.kVelocity);
      m_updateClawPIDPub.set(false);
    }

    if (m_updateClawPIDSub.get())
    {
      m_clawShootConfig.closedLoop.pid(m_clawkPSub.get(), m_clawkISub.get(), m_clawkDSub.get());
      m_clawShootSparkMax.configure(m_clawShootConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      m_updateClawPIDPub.set(false);
    }
  }

  public double setClawShooterVelocity (double targetVelocity)
  {
    m_clawShootController.setReference(targetVelocity, ControlType.kVelocity);
    m_lastTargetVelocity = targetVelocity;
    return m_lastTargetVelocity;
  }

  public boolean isAtTargetShootVelocity ()
  {
    return Math.abs(m_lastTargetVelocity - m_clawShootEncoder.getPosition()) < 10;
  }
  
  public void intakeAlgae ()
  {
    m_clawIntakeSparkMax.set(m_clawIntakeSpeed);
  }

  public void feedAlgae ()
  {
    m_clawIntakeSparkMax.set(-m_clawIntakeSpeed);
  }

  public void extendSolenoid ()
  {
    m_solenoid.set(true);
  }

  public void retractSolenoid ()
  {
    m_solenoid.set(false);
  }
}
