// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LiftSubsystem extends SubsystemBase {
  
  public enum liftTargetLevels
  {
      L4,
      L3,
      L2,
      Base,
      L4SlowSpeedMin,
      Processor,
      HighAlage,
      LowAlage,
      AlgaeRemovalPrep
  };

  private final SparkMax m_lift = new SparkMax(RobotMap.kLiftLeadMotor, MotorType.kBrushless);
  private final SparkMax m_liftFollower = new SparkMax(RobotMap.kLiftFollowerMotor, MotorType.kBrushless);
  private final SparkMaxConfig m_config;
  private SparkClosedLoopController m_liftController;
  private SparkMaxAlternateEncoder m_liftEncoder;
  private double m_lastTargetPosition;

  private final double kLiftP = 1.0;
  private final double kLiftI = 0;
  private final double kLiftD = 0;
  private final double kLiftFF = 0;

  private static final double kL4Position = 9.1;
  private static final double kL3Position = 4.7;
  private static final double kL2Position = 1.5;
  private static final double kBasePosition = 0.22;
  private static final double kL4SlowSpeedMin = 8.0;
  private static final double kProcessorPosition = 0.7;
  private static final double kHighAlagePosition = 7;
  private static final double kLowAlagePosition = 5;
  private static final double kAlageRemovalPrepPosition = 3;

  private boolean m_initalized = false;

  private final DoublePublisher m_sparkLiftRotations;
  private final DoublePublisher m_sparkVelocity;
  private final DoublePublisher m_appliedOutputPub;
  private final DoublePublisher m_lastTargetPub;
  private final DoubleSubscriber m_targetRotations;
  private final DoubleSubscriber m_liftP;
  private final DoubleSubscriber m_liftI;
  private final DoubleSubscriber m_liftD;
  private final DoubleSubscriber m_liftFF;
  private final BooleanSubscriber m_goToTargetRotationsSub;
  private final BooleanPublisher m_goToTargetRotationsPub;
  private final BooleanPublisher m_updateLiftPIDPub;
  private final BooleanSubscriber m_updateLiftPIDSub;
  
  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {  
    final String name = "LiftSubsystem";
   
    m_config = new SparkMaxConfig();
    m_config.idleMode(IdleMode.kBrake);
    m_config.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(kLiftP, kLiftI, kLiftD);
    var alternateEncoderConfig = new AlternateEncoderConfig();
    alternateEncoderConfig.setSparkMaxDataPortConfig();
    alternateEncoderConfig.inverted(true);
    m_config.apply(alternateEncoderConfig);
   
    m_lift.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_liftEncoder = (SparkMaxAlternateEncoder) m_lift.getAlternateEncoder(); 
    m_liftController = m_lift.getClosedLoopController();

    var followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake).follow(RobotMap.kLiftLeadMotor, true);
    m_liftFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("lift Subsystem");

    m_sparkLiftRotations = datatable.getDoubleTopic(name + "/Rotations").publish();
    m_sparkVelocity = datatable.getDoubleTopic(name + "/Velocity").publish();
    m_appliedOutputPub = datatable.getDoubleTopic(name + "/Applied Output").publish();
    m_lastTargetPub = datatable.getDoubleTopic("Last Target").publish();

    var targetRotations = datatable.getDoubleTopic(name + "/target rotations");
    targetRotations.publish().set(0);
    m_targetRotations = targetRotations.subscribe(0.0);
   
    var liftP = datatable.getDoubleTopic(name + "liftP");
    liftP.publish().set(kLiftP);
    m_liftP = liftP.subscribe(kLiftP);

    var liftI = datatable.getDoubleTopic(name + "liftI");
    liftI.publish().set(kLiftI);
    m_liftI = liftI.subscribe(kLiftI);

    var liftD = datatable.getDoubleTopic(name + "liftD");
    liftD.publish().set(kLiftD);
    m_liftD = liftD.subscribe(kLiftD);

    var liftFF = datatable.getDoubleTopic(name + "liftFF");
    liftFF.publish().set(kLiftD);
    m_liftFF = liftFF.subscribe(kLiftFF);

    var goToTargetRotations = datatable.getBooleanTopic(name + "/goToTargetRotations");
    m_goToTargetRotationsPub = goToTargetRotations.publish();
    m_goToTargetRotationsPub.set(false);
    m_goToTargetRotationsSub = goToTargetRotations.subscribe(false);

    var updateliftPIDTopic = datatable.getBooleanTopic("update Steer PID");
    m_updateLiftPIDPub = updateliftPIDTopic.publish();
    m_updateLiftPIDPub.set(false);
    m_updateLiftPIDSub = updateliftPIDTopic.subscribe(false);
  }

  public void initialize()
  {
    if (m_initalized)
        return;
    setTargetPosition(liftTargetLevels.Base);
    m_initalized = true;
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ticks++;
    if (m_ticks % 11 != 5)
        return;

    dashboardUpdate();
  }

  public void dashboardUpdate() {
    m_sparkLiftRotations.set(m_liftEncoder.getPosition());
    m_sparkVelocity.set(m_liftEncoder.getVelocity());
    m_appliedOutputPub.set(m_lift.getAppliedOutput());

    if(m_goToTargetRotationsSub.get())
    {
      goToTargetPosition(m_targetRotations.get());
      m_goToTargetRotationsPub.set(false);
    }

    if (m_updateLiftPIDSub.get())
    {
      var newP = m_liftP.get();
      var newI = m_liftI.get();
      var newD = m_liftD.get();
      var newFF = m_liftFF.get();

      m_config.closedLoop.pidf(newP, newI, newD, newFF);
      m_lift.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_updateLiftPIDPub.set(false);
    }
  }

  public void directDrive(double power) 
  {
    m_lift.set(power);
  }

  public double setTargetPosition(liftTargetLevels target)
  {
    var targetPosition = LiftPositionValue(target);
    goToTargetPosition(targetPosition);
    m_lastTargetPosition = targetPosition;
    m_lastTargetPub.set(m_lastTargetPosition);
    return m_lastTargetPosition;
  }

  private void goToTargetPosition(double target)
  {
    m_liftController.setReference(target, SparkMax.ControlType.kPosition);
  }

  private static double LiftPositionValue(liftTargetLevels target)
  {
    switch (target) {
      case L4:
        return kL4Position;
      case L3:
        return kL3Position;
      case L2:
        return kL2Position;
      case L4SlowSpeedMin:
        return kL4SlowSpeedMin;
      case Processor:
        return kProcessorPosition;
      case HighAlage:
        return kHighAlagePosition;
      case LowAlage:
        return kLowAlagePosition;
      case AlgaeRemovalPrep:
        return kAlageRemovalPrepPosition;
      default:
        return kBasePosition;  // loading level for wall
    }
  }

  public boolean isAtTargetPosition()
  {
    return Math.abs(m_lastTargetPosition - m_liftEncoder.getPosition()) < 0.05;
  }

  public boolean isAbovePosition(liftTargetLevels level)
  {
    return m_liftEncoder.getPosition() >  LiftPositionValue(level);
  }

  public void stopAtCurrentPosition()
  {
    goToTargetPosition(m_liftEncoder.getPosition());
  }

  public Command stopAtCurrentPositionCommand()
  {
    return this.startEnd(() -> this.stopAtCurrentPosition(), () -> {});
  }
}
