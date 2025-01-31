// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LiftSubsystem.liftTargetPositions;

public class CoralDeliverySubsystem extends SubsystemBase {
  /** Creates a new CoralDeliverySubsystem. */
  public enum coralIndexTargetPositions 
  {
    Left,
    Right,
    Center
  };

  private final SparkMax m_indexSparkMax = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax m_rightCoralControlSparkMax = new SparkMax(22, MotorType.kBrushless);
  private final SparkMax m_leftCoralControlSparkMax = new SparkMax(23, MotorType.kBrushless);
  private SparkRelativeEncoder m_indexEncoder;
  private SparkRelativeEncoder m_rightCoralControlEncoder;
  private SparkRelativeEncoder m_leftCoralControlEncoder;
  private final SparkMaxConfig m_indexConfig;
  private final SparkMaxConfig m_rightCoralControlConfig;
  private final SparkMaxConfig m_leftCoralControlConfig;
  private SparkClosedLoopController m_indexController;

  private final double kIndexP = 0;
  private final double kIndexI = 0;
  private final double kIndexD = 0;
  private double m_lastTargetPosition;

  public CoralDeliverySubsystem() {
 
    m_indexConfig = new SparkMaxConfig();
    m_indexConfig.idleMode(IdleMode.kBrake);
    m_indexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kIndexP, kIndexI, kIndexD);
    
    m_rightCoralControlConfig = new SparkMaxConfig();
    m_rightCoralControlConfig.idleMode(IdleMode.kCoast);
    m_leftCoralControlConfig = new SparkMaxConfig();
    m_leftCoralControlConfig.idleMode(IdleMode.kCoast);

    m_indexSparkMax.configure(m_indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightCoralControlSparkMax.configure(m_rightCoralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftCoralControlSparkMax.configure(m_leftCoralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_indexEncoder = (SparkRelativeEncoder) m_indexSparkMax.getEncoder(); 
    m_rightCoralControlEncoder = (SparkRelativeEncoder) m_rightCoralControlSparkMax.getEncoder(); 
    m_leftCoralControlEncoder = (SparkRelativeEncoder) m_leftCoralControlSparkMax.getEncoder(); 

    m_indexController = m_indexSparkMax.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double setTargetIndexPosition (coralIndexTargetPositions targetIndexPosition)
  {
    var targetPosition = CoralIndexPositionValue(targetIndexPosition);
    m_indexController.setReference(targetPosition, SparkMax.ControlType.kPosition);
    m_lastTargetPosition = targetPosition;
    return m_lastTargetPosition;
  }

  private static int CoralIndexPositionValue(coralIndexTargetPositions target)
  {
    switch (target) {
      case Left:
        return -45; //random numbers
      case Right:
        return 45;
      case Center:
        return 0;
      default:
        return 0;
    }
  }
  
  public boolean isAtIndexTargetPosition()
  {
    return Math.abs(m_lastTargetPosition - m_indexEncoder.getPosition()) < 10;
  }

  public void stopAtIndexCurrentPosition()
  {
    m_indexController.setReference(m_indexEncoder.getPosition(), SparkMax.ControlType.kPosition);
  }

  public void setLeftCoralControlVelocity(double targetVelocity)
  {
    m_leftCoralControlSparkMax.set(targetVelocity);
  }

  public void setRightCoralControlVelocity(double targetVelocity)
  {
    m_rightCoralControlSparkMax.set(targetVelocity);
  }
}
