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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new AlgaeClawSubsystem. */
  private final SparkMax m_clawFeedSparkMax = new SparkMax(RobotMap.kAlgaeClawIntakeMotor, MotorType.kBrushless);
  private final SparkMax m_clawArmExtendSparkMax = new SparkMax(RobotMap.kAlgaeArmExtendMotor, MotorType.kBrushless);
  private SparkClosedLoopController m_clawArmExtendController;
  private final SparkMaxConfig m_clawFeedConfig;
  private final SparkMaxConfig m_clawArmExtendConfig;
  private SparkRelativeEncoder m_clawArmExtendEncoder;

  private double m_clawFeedMotorSpeed = 0.2;

  private double kClawExtendPosition = 10.0;
  private double kClawRetractPosition = -1.0;
  private double kClawTransportPosition = 0.0;

  private final double kClawArmP = 0.08;
  private final double kClawArmI = 0;
  private final double kClawArmD = 0;
  private final double kClawArmFF = 0;

  private final DoubleSubscriber m_clawFeedSpeedSub;
  private final BooleanSubscriber m_clawFeedGoToSpeedSub;
  private final BooleanPublisher m_clawFeedGoToSpeedPub;
  
  private final DoublePublisher m_armExtendMotorPositionPub;
  private final DoublePublisher m_armExtendMotorVelocityPub;
  private final DoublePublisher m_armExtendAppliedOutputPub;
  private final DoubleSubscriber m_armExtendPSub;
  private final DoubleSubscriber m_armExtendISub;
  private final DoubleSubscriber m_armExtendDSub;
  private final DoubleSubscriber m_armExtendFFSub;
  private final BooleanSubscriber m_updateArmExtendPIDSub;
  private final BooleanPublisher m_updateArmExtendPIDPub;
  private final DoubleSubscriber m_armExtendTargetPositionSub;
  private final BooleanSubscriber m_goToArmExtendTargetPositionSub;
  private final BooleanPublisher m_goToArmExtendTargetPositionPub;

  public AlgaeClawSubsystem() {
    final String name = "Algae Claw Subsystem";

    m_clawFeedConfig = new SparkMaxConfig();
    m_clawFeedConfig.idleMode(IdleMode.kBrake);

    m_clawArmExtendConfig = new SparkMaxConfig();
    m_clawArmExtendConfig.idleMode(IdleMode.kBrake);
    m_clawArmExtendConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kClawArmP, kClawArmI, kClawArmD, kClawArmFF);

    m_clawFeedSparkMax.configure(m_clawFeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_clawArmExtendSparkMax.configure(m_clawArmExtendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_clawArmExtendEncoder = (SparkRelativeEncoder) m_clawArmExtendSparkMax.getEncoder();

    m_clawArmExtendController = m_clawArmExtendSparkMax.getClosedLoopController();

    // start with the claws to the retract state
    retractClawArms();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("AlgaeClawSubsystem");

    var clawFeedSpeedSub = datatable.getDoubleTopic(name + "/ClawFeedSpeed");
    clawFeedSpeedSub.publish().set(m_clawFeedMotorSpeed);
    m_clawFeedSpeedSub = clawFeedSpeedSub.subscribe(0.0);

    var clawFeedGoToSpeed = datatable.getBooleanTopic(name + "go To Feed Motor Speed");
    m_clawFeedGoToSpeedPub = clawFeedGoToSpeed.publish();
    m_clawFeedGoToSpeedPub.set(false);
    m_clawFeedGoToSpeedSub = clawFeedGoToSpeed.subscribe(false);

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

    var clawArmFFSub = datatable.getDoubleTopic(name + "Arm FF");
    clawArmFFSub.publish().set(kClawArmFF);
    m_armExtendFFSub = clawArmFFSub.subscribe(kClawArmFF);

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

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_armExtendMotorPositionPub.set(m_clawArmExtendEncoder.getPosition());

    m_ticks++;
    if (m_ticks % 13 != 7)
       return;

    // dashboardUpdate();
  }

  private void dashboardUpdate ()
  {
    m_armExtendMotorVelocityPub.set(m_clawArmExtendEncoder.getVelocity());
    m_armExtendAppliedOutputPub.set(m_clawArmExtendSparkMax.getAppliedOutput());

    if (m_clawFeedGoToSpeedSub.get())
    {
      m_clawFeedMotorSpeed = m_clawFeedSpeedSub.get();
      m_clawFeedSparkMax.set(m_clawFeedMotorSpeed);
      m_clawFeedGoToSpeedPub.set(false);
    }

    if (m_updateArmExtendPIDSub.get())
    {
      var newP = m_armExtendPSub.get();
      var newI = m_armExtendISub.get();
      var newD = m_armExtendDSub.get();
      var newFf = m_armExtendFFSub.get();

      m_clawArmExtendConfig.closedLoop.pidf(newP, newI, newD, newFf);
      m_clawArmExtendSparkMax.configure(m_clawArmExtendConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_updateArmExtendPIDPub.set(false);
    }

    if (m_goToArmExtendTargetPositionSub.get())
    {
      setExtendArmPosition(m_armExtendTargetPositionSub.get());
      m_goToArmExtendTargetPositionPub.set(false);
    }
  }

  public void intakeAlgae()
  {
    setAlgaeFeedMotorSpeed(-m_clawFeedMotorSpeed);
  }
  
  public void setAlgaeFeedMotorSpeed(double targetSpeed)
  {
    m_clawFeedSparkMax.set(targetSpeed);
  }

  public void feedAlgaeIntoProcessor()
  {
    setAlgaeFeedMotorSpeed(m_clawFeedMotorSpeed);
  }

  public void extendClawArms()
  {
    setExtendArmPosition(kClawExtendPosition);
  }

  public void retractClawArms()
  {
    setExtendArmPosition(kClawRetractPosition);
  }

  public void goToClawTransportPosition()
  {
    setExtendArmPosition(kClawTransportPosition);
  }

  private void setExtendArmPosition(double target)
  {
     m_clawArmExtendController.setReference(target, ControlType.kPosition);
  }

  public void stopClawWheels()
  {
    m_clawFeedSparkMax.stopMotor();
  }

  public Command stopClawWheelsCommand()
  {
    return this.startEnd(() -> this.stopClawWheels(), () -> {});
  }
}
