// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LiftSubsystem extends SubsystemBase {
  
  public enum liftTargetPositions
  {
      L4,
      L3,
      L2,
      Base
  };

  private final SparkMax m_lift = new SparkMax(RobotMap.kLiftLeadMotor, MotorType.kBrushless);
  private final SparkMax m_liftFollower = new SparkMax(RobotMap.kLiftFollowerMotor, MotorType.kBrushless);
  private final SparkMaxConfig m_config;
  private SparkClosedLoopController m_liftController;
  private SparkMaxAlternateEncoder m_liftEncoder;
  private double m_lastTargetPosition;
  private final DigitalInput m_liftDetect = new DigitalInput(RobotMap.kLiftDetectInput);

  private final double kLiftP = 1.0;
  private final double kLiftI = 0;
  private final double kLiftD = 0;
  private final double kLiftFF = 0;
  private final double kMaxVelocity = 0;
  private final double kMaxAcceleration = 1;

  private final DoublePublisher m_sparkLiftRotations;
  private final DoublePublisher m_sparkLiftCurrent;
  private final DoublePublisher m_sparkVelocity;
  private final BooleanPublisher m_mechanicalSwitchPub;
  private final DoubleSubscriber m_targetRotations;
  private final DoubleSubscriber m_liftP;
  private final DoubleSubscriber m_liftI;
  private final DoubleSubscriber m_liftD;
  private final DoubleSubscriber m_liftFF;
  private final DoubleSubscriber m_maxVelocitySub;
  private final DoubleSubscriber m_maxAccelerationSub;
  private final BooleanSubscriber m_goToTargetRotationsSub;
  private final BooleanPublisher m_goToTargetRotationsPub;
  private final BooleanPublisher m_updateLiftPIDPub;
  private final BooleanSubscriber m_updateLiftPIDSub;
  private final BooleanSubscriber m_slot1Sub;
  
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
    m_sparkLiftCurrent = datatable.getDoubleTopic(name + "/Current").publish();
    m_sparkVelocity = datatable.getDoubleTopic(name + "/Velocity").publish();
    m_mechanicalSwitchPub = datatable.getBooleanTopic(name + "/Mechanical Switch Value").publish();

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

    var maxVelocitySub = datatable.getDoubleTopic(name + "Max Velocity");
    maxVelocitySub.publish().set(kMaxVelocity);
    m_maxVelocitySub = maxVelocitySub.subscribe(kMaxVelocity);

    var maxAccelerationSub = datatable.getDoubleTopic(name + "Max Acceleration");
    maxAccelerationSub.publish().set(kMaxAcceleration);
    m_maxAccelerationSub = maxAccelerationSub.subscribe(kMaxAcceleration);

    var goToTargetRotations = datatable.getBooleanTopic(name + "/goToTargetRotations");
    m_goToTargetRotationsPub = goToTargetRotations.publish();
    m_goToTargetRotationsPub.set(false);
    m_goToTargetRotationsSub = goToTargetRotations.subscribe(false);

    var updateliftPIDTopic = datatable.getBooleanTopic("update Steer PID");
    m_updateLiftPIDPub = updateliftPIDTopic.publish();
    m_updateLiftPIDPub.set(false);
    m_updateLiftPIDSub = updateliftPIDTopic.subscribe(false);

    var slot1Topic = datatable.getBooleanTopic("control Slot 1");
    slot1Topic.publish().set(false);
    m_slot1Sub = slot1Topic.subscribe(false);
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ticks++;
    // if (m_ticks % 11 != 5)
    //     return;

    dashboardUpdate();
  }

  public void dashboardUpdate() {
    m_sparkLiftRotations.set(m_liftEncoder.getPosition());
    m_sparkLiftCurrent.set(m_lift.getAppliedOutput());
    m_sparkVelocity.set(m_liftEncoder.getVelocity());
    m_mechanicalSwitchPub.set(isLiftAtBottom());

    var slot1 = m_slot1Sub.get();

    if(m_goToTargetRotationsSub.get())
    {
      double target = m_targetRotations.get();
      if (slot1)
      {
        m_liftController.setReference(target, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot1);
      }
      else{
        goToTargetPosition(target);
      }
      m_goToTargetRotationsPub.set(false);
    }

    if (m_updateLiftPIDSub.get())
    {
      var newP = m_liftP.get();
      var newI = m_liftI.get();
      var newD = m_liftD.get();
      var newFF = m_liftFF.get();
      var newMaxVelocity = m_maxVelocitySub.get();
      var newMaxAccceleration = m_maxAccelerationSub.get();

      m_config.closedLoop.pidf(newP, newI, newD, newFF, slot1 ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0);
      var updatedMaxMotion = new MAXMotionConfig().maxVelocity(newMaxVelocity).maxAcceleration(newMaxAccceleration);
      m_config.closedLoop.apply(updatedMaxMotion);
      m_lift.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_updateLiftPIDPub.set(false);
    }
  }

  public void directDrive(double power) 
  {
    m_lift.set(power);
  }

  public double setTargetPosition(liftTargetPositions target)
  {
    var targetPosition = LiftPositionValue(target);
    goToTargetPosition(targetPosition);
    m_lastTargetPosition = targetPosition;
    return m_lastTargetPosition;
  }

  private void goToTargetPosition(double target)
  {
    m_liftController.setReference(target, SparkMax.ControlType.kPosition);
  }

  private static int LiftPositionValue(liftTargetPositions target)
  {
    switch (target) {
      case L4:
        return 4;
      case L3:
        return 3;
      case L2:
        return 2;
      default:
        return 1;
    }
  }

  public boolean isAtTargetPosition()
  {
    return Math.abs(m_lastTargetPosition - m_liftEncoder.getPosition()) < 10;
  }

  public void stopAtCurrentPosition()
  {
    goToTargetPosition(m_liftEncoder.getPosition());
  }

  public boolean isLiftAtBottom()
  {
    return m_liftDetect.get();
  }
}
