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
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new AlgaeClawSubsystem. */
  private final SparkMax m_clawShootSparkMax = new SparkMax(RobotMap.kAlgaeClawShootMotor, MotorType.kBrushless);
  private final SparkMax m_clawFeedSparkMax = new SparkMax(RobotMap.kAlgaeClawIntakeMotor, MotorType.kBrushless);
  private final SparkMax m_clawArmExtendSparkMax = new SparkMax(RobotMap.kAlgaeArmExtendMotor, MotorType.kBrushed);
  private SparkClosedLoopController m_clawShootController;
  private SparkClosedLoopController m_clawArmExtendController;
  private SparkRelativeEncoder m_clawShootEncoder;
  private SparkMaxAlternateEncoder m_clawArmExtendEncoder;
  private final SparkMaxConfig m_clawShootConfig;
  private final SparkMaxConfig m_clawFeedConfig;
  private final SparkMaxConfig m_clawArmExtendConfig;

  private double m_clawFeedMotorSpeed = 0.2; //random number
  private double m_clawShootSpeed = 5000; //random number

  private double kClawExtendPosition = 0;
  private double kClawRetractPosition = 0;

  private final double kClawShootP = 0;
  private final double kClawShootI = 0;
  private final double kClawShootD = 0;
  private final double kClawArmP = 0;
  private final double kClawArmI = 0;
  private final double kClawArmD = 0;

  private final DoubleSubscriber m_clawFeedSpeedSub;
  private final DoubleSubscriber m_clawShootRPMSub;
  private final BooleanSubscriber m_clawFeedGoToSpeedSub;
  private final BooleanPublisher m_clawFeedGoToSpeedPub;
  private final BooleanSubscriber m_clawShootGoToSpeedSub;
  private final BooleanPublisher m_clawShootGoToSpeedPub;
  private final DoubleSubscriber m_ClawShootPSub;
  private final DoubleSubscriber m_ClawShootISub;
  private final DoubleSubscriber m_ClawShootDSub;
  private final BooleanSubscriber m_updateClawShootPIDSub;
  private final BooleanPublisher m_updateClawShootPIDPub;
  private final DoublePublisher m_clawShootSpeedEncoderReadingPub;
  
  private final DoublePublisher m_armExtendMotorPositionPub;
  private final DoublePublisher m_armExtendMotorVelocityPub;
  private final DoublePublisher m_armExtendAppliedOutputPub;
  private final DoubleSubscriber m_armExtendPSub;
  private final DoubleSubscriber m_armExtendISub;
  private final DoubleSubscriber m_armExtendDSub;
  private final BooleanSubscriber m_updateArmExtendPIDSub;
  private final BooleanPublisher m_updateArmExtendPIDPub;
  private final DoubleSubscriber m_armExtendTargetPositionSub;
  private final BooleanSubscriber m_goToArmExtendTargetPositionSub;
  private final BooleanPublisher m_goToArmExtendTargetPositionPub;

  public AlgaeClawSubsystem() {
    final String name = "Algae Claw Subsystem";

    m_clawShootConfig = new SparkMaxConfig();
    m_clawShootConfig.idleMode(IdleMode.kBrake);
    m_clawShootConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kClawShootP, kClawShootI, kClawShootD);

    m_clawFeedConfig = new SparkMaxConfig();
    m_clawFeedConfig.idleMode(IdleMode.kBrake);

    m_clawArmExtendConfig = new SparkMaxConfig();
    m_clawArmExtendConfig.idleMode(IdleMode.kBrake);
    m_clawArmExtendConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(kClawArmP, kClawArmI, kClawArmD);
    var alternateEncoderConfig = new AlternateEncoderConfig();
    alternateEncoderConfig.setSparkMaxDataPortConfig();
    m_clawArmExtendConfig.apply(alternateEncoderConfig);

    m_clawShootSparkMax.configure(m_clawShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_clawFeedSparkMax.configure(m_clawFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_clawArmExtendSparkMax.configure(m_clawArmExtendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_clawShootEncoder = (SparkRelativeEncoder) m_clawShootSparkMax.getEncoder(); 
    m_clawShootController = m_clawShootSparkMax.getClosedLoopController();

    m_clawArmExtendEncoder = (SparkMaxAlternateEncoder)m_clawArmExtendSparkMax.getAlternateEncoder();
    m_clawArmExtendController = m_clawArmExtendSparkMax.getClosedLoopController();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("AlgaeClawSubsystem");

    var clawFeedSpeedSub = datatable.getDoubleTopic(name + "/ClawFeedSpeed");
    clawFeedSpeedSub.publish().set(0);
    m_clawFeedSpeedSub = clawFeedSpeedSub.subscribe(0.0);

    var clawShootRPMSub = datatable.getDoubleTopic(name + "/ClawShootSpeed");
    clawShootRPMSub.publish().set(0);
    m_clawShootRPMSub = clawShootRPMSub.subscribe(0.0);

    var clawFeedGoToSpeed = datatable.getBooleanTopic("go To Feed Motor Speed");
    m_clawFeedGoToSpeedPub = clawFeedGoToSpeed.publish();
    m_clawFeedGoToSpeedPub.set(false);
    m_clawFeedGoToSpeedSub = clawFeedGoToSpeed.subscribe(false);

    var clawShootGoToSpeed = datatable.getBooleanTopic("go To Shoot Motor Speed");
    m_clawShootGoToSpeedPub = clawShootGoToSpeed.publish();
    m_clawShootGoToSpeedPub.set(false);
    m_clawShootGoToSpeedSub = clawShootGoToSpeed.subscribe(false);

    var clawShootPSub = datatable.getDoubleTopic(name + "Shoot P");
    clawShootPSub.publish().set(kClawShootP);
    m_ClawShootPSub = clawShootPSub.subscribe(kClawShootP);

    var clawShootISub = datatable.getDoubleTopic(name + "Shoot I");
    clawShootISub.publish().set(kClawShootI);
    m_ClawShootISub = clawShootISub.subscribe(kClawShootI);

    var clawShootDSub = datatable.getDoubleTopic(name + "Shoot D");
    clawShootDSub.publish().set(kClawShootD);
    m_ClawShootDSub = clawShootDSub.subscribe(kClawShootD);

    var updateClawShootPID = datatable.getBooleanTopic(name + "update PID");
    m_updateClawShootPIDPub = updateClawShootPID.publish();
    m_updateClawShootPIDPub.set(false);
    m_updateClawShootPIDSub = updateClawShootPID.subscribe(false);

    m_clawShootSpeedEncoderReadingPub = datatable.getDoubleTopic(name + "Shoot Encoder Reading").publish();

    m_armExtendMotorPositionPub = datatable.getDoubleTopic(name + "Arm Position").publish();
    m_armExtendMotorVelocityPub = datatable.getDoubleTopic(name + "Arm Velocity").publish();
    m_armExtendAppliedOutputPub = datatable.getDoubleTopic(name + "Arm Applied Output").publish();

    var clawArmPSub = datatable.getDoubleTopic(name + "Arm P");
    clawArmPSub.publish().set(kClawArmP);
    m_armExtendPSub = clawArmPSub.subscribe(kClawArmP);

    var clawArmISub = datatable.getDoubleTopic(name + "Arm I");
    clawArmISub.publish().set(kClawArmI);
    m_armExtendISub = clawArmISub.subscribe(kClawArmI);

    var clawArmDSub = datatable.getDoubleTopic(name + "Arm D");
    clawArmDSub.publish().set(kClawArmD);
    m_armExtendDSub = clawArmDSub.subscribe(kClawArmD);

    var clawTargetPositionSub = datatable.getDoubleTopic(name + "Arm Target Position");
    clawTargetPositionSub.publish().set(0);
    m_armExtendTargetPositionSub = clawTargetPositionSub.subscribe(0);

    var clawArmGoToPosition = datatable.getBooleanTopic(name + "go To Arm Position");
    m_goToArmExtendTargetPositionPub = clawArmGoToPosition.publish();
    m_goToArmExtendTargetPositionPub.set(false);
    m_goToArmExtendTargetPositionSub = clawArmGoToPosition.subscribe(false);

    var updateArmPID = datatable.getBooleanTopic(name + "update Arm PID");
    m_updateArmExtendPIDPub = updateArmPID.publish();
    m_updateArmExtendPIDPub.set(false);
    m_updateArmExtendPIDSub = updateArmPID.subscribe(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dashboardUpdate();
  }

  private void dashboardUpdate ()
  {
    m_clawShootSpeedEncoderReadingPub.set(m_clawShootEncoder.getVelocity());
    m_armExtendMotorPositionPub.set(m_clawArmExtendEncoder.getPosition());
    m_armExtendMotorVelocityPub.set(m_clawArmExtendEncoder.getVelocity());
    m_armExtendAppliedOutputPub.set(m_clawArmExtendSparkMax.getAppliedOutput());

    if (m_clawFeedGoToSpeedSub.get())
    {
      m_clawFeedMotorSpeed = m_clawFeedSpeedSub.get();
      setAlgaeFeedMotorSpeed (m_clawFeedMotorSpeed);
      m_clawFeedGoToSpeedPub.set(false);
    }

    if (m_clawShootGoToSpeedSub.get())
    {
      m_clawShootSpeed = m_clawShootRPMSub.get();
      setClawShootSpeed(m_clawShootSpeed);
      m_clawShootGoToSpeedPub.set(false);
    }

    if (m_updateClawShootPIDSub.get())
    {
      var newP = m_ClawShootPSub.get();
      var newI = m_ClawShootISub.get();
      var newD = m_ClawShootDSub.get();

      m_clawShootConfig.closedLoop.pid(newP, newI, newD);
      m_clawShootSparkMax.configure(m_clawShootConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_updateClawShootPIDPub.set(false);
    }
  }

  public void setClawShootSpeed()
  {
    setClawShootSpeed(m_clawShootSpeed);
  }

  private void setClawShootSpeed(double targetRPM)
  {
    m_clawShootController.setReference(targetRPM, ControlType.kVelocity);
  }

  public void intakeAlgae ()
  {
    m_clawShootSparkMax.set(-m_clawFeedMotorSpeed);
    setAlgaeFeedMotorSpeed(-m_clawFeedMotorSpeed);
  }
  
  public void setAlgaeFeedMotorSpeed (double targetSpeed)
  {
    m_clawFeedSparkMax.set(targetSpeed);
  }

  public void feedAlgaeIntoProcessor ()
  {
    setClawShootSpeed();
    setAlgaeFeedMotorSpeed(m_clawFeedMotorSpeed);
  }

  public void extendClawArms ()
  {
    setExtendArmPosition(kClawExtendPosition);
  }

  public void retractClawArms ()
  {
    setExtendArmPosition(kClawRetractPosition);
  }

  private void setExtendArmPosition (double target)
  {
    m_clawArmExtendController.setReference(target, ControlType.kPosition);
  }

  public void stopClawShootMotor ()
  {
    m_clawShootSparkMax.stopMotor();
  }

  public void stopClawIntakeMotor ()
  {
    m_clawFeedSparkMax.stopMotor();
  }
}
