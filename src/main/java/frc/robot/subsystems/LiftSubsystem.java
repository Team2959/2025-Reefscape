// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
  public enum liftTargetPositions
  {
      L4,
      L3,
      L2,
      Base
  };

  private final SparkMax m_lift = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax m_liftFollower = new SparkMax(6, MotorType.kBrushless);
  private SparkClosedLoopController m_liftController;
  private SparkRelativeEncoder m_liftEncoder;
  private double m_lastTargetPosition;

  private final double kLiftP = 0;
  private final double kLiftI = 0;
  private final double kLiftD = 0;

  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kLiftP, kLiftI, kLiftD);
   
    m_lift.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_liftEncoder = (SparkRelativeEncoder) m_lift.getEncoder(); 
    m_liftController = m_lift.getClosedLoopController();

    var followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake)
      .follow(5, true);
    m_liftFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void directDrive(double power) 
  {
    m_lift.set(power);
  }

  public double setTargetPosition(liftTargetPositions target)
  {
    var targetPosition = LiftPositionValue(target);
    m_liftController.setReference(targetPosition, SparkMax.ControlType.kPosition);
    m_lastTargetPosition = targetPosition;
    return m_lastTargetPosition;
  }

  private static int LiftPositionValue(liftTargetPositions target)
  {
    switch (target) {
      case L4:
        return 1000;
      case L3:
        return 750;
      case L2:
        return 500;
      default:
        return 50;
    }
  }

  public boolean isAtTargetPosition()
  {
    return Math.abs(m_lastTargetPosition - m_liftEncoder.getPosition()) < 10;
  }

  public void stopAtCurrentPosition()
  {
    m_liftController.setReference(m_liftEncoder.getPosition(), SparkMax.ControlType.kPosition);
  }
}
