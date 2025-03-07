// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
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

public class CoralIndexSubsystem extends SubsystemBase {
  /** Creates a new CoralIndexSubsystem. */
  public enum CoralIndexTargetPositions 
  {
    Left,
    Right,
    Center
  };

  private final SparkMax m_indexSparkMax = new SparkMax(RobotMap.kCoralDeliveryIndexMotor, MotorType.kBrushed);
  private final SparkMaxAlternateEncoder m_indexEncoder;
  private final SparkMaxConfig m_indexConfig;
  private final SparkClosedLoopController m_indexController;

  private final double kIndexP = 1.8;
  private final double kIndexI = 0.0;
  private final double kIndexD = 0;
  private final double kIndexFf = 0.0;
  private double m_lastTargetPosition;

  private static final double kLeftPosition = -7.37;
  private static final double kRightPosition = -0.1;
  private static final double kCenterPosition = -3.7;

  private final DoublePublisher m_indexPositionPub;
  private final DoublePublisher m_indexAppliedOutputPub;
  private final DoubleSubscriber m_indexPSub;
  private final DoubleSubscriber m_indexISub;
  private final DoubleSubscriber m_indexDSub;
  private final DoubleSubscriber m_indexFFSub;
  private final BooleanPublisher m_updateIndexPIDPub;
  private final BooleanSubscriber m_updateIndexPIDSub;
  private final DoubleSubscriber m_indexTargetRotationsSub;
  private final BooleanSubscriber m_goToTargetIndexPositionSub;
  private final BooleanPublisher m_goToTargetIndexPositionPub;

  public CoralIndexSubsystem() {
    m_indexConfig = new SparkMaxConfig();
    m_indexConfig.idleMode(IdleMode.kBrake);
    m_indexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pidf(kIndexP, kIndexI, kIndexD, kIndexFf);
    var alternateEncoderConfig = new AlternateEncoderConfig();
    alternateEncoderConfig.setSparkMaxDataPortConfig();
    m_indexConfig.apply(alternateEncoderConfig);

    m_indexSparkMax.configure(m_indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_indexEncoder = (SparkMaxAlternateEncoder)m_indexSparkMax.getAlternateEncoder();
    m_indexController = m_indexSparkMax.getClosedLoopController();

    final String name = "Coral Index Subsystem";
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable(name);

    m_indexPositionPub = datatable.getDoubleTopic("Index Position").publish();
    m_indexAppliedOutputPub = datatable.getDoubleTopic("Index Applied Output").publish();
 
    var indexPSub = datatable.getDoubleTopic("index P");
    indexPSub.publish().set(kIndexP);
    m_indexPSub = indexPSub.subscribe(kIndexP);

    var indexISub = datatable.getDoubleTopic("index I");
    indexISub.publish().set(kIndexI);
    m_indexISub = indexISub.subscribe(kIndexI);

    var indexDSub = datatable.getDoubleTopic("index D");
    indexDSub.publish().set(kIndexD);
    m_indexDSub = indexDSub.subscribe(kIndexD);

    var indexFFSub = datatable.getDoubleTopic("index FF");
    indexFFSub.publish().set(kIndexFf);
    m_indexFFSub = indexFFSub.subscribe(kIndexFf);

    var updateIndexPIDPub = datatable.getBooleanTopic("update Index PID");
    m_updateIndexPIDPub = updateIndexPIDPub.publish();
    m_updateIndexPIDPub.set(false);
    m_updateIndexPIDSub = updateIndexPIDPub.subscribe(false);

    var indexTargetPositionSub = datatable.getDoubleTopic("Index Target Position");
    indexTargetPositionSub.publish().set(0);
    m_indexTargetRotationsSub = indexTargetPositionSub.subscribe(0);

    var goToTargetIndexPosition = datatable.getBooleanTopic("Go To Target Index Position");
    m_goToTargetIndexPositionPub = goToTargetIndexPosition.publish();
    m_goToTargetIndexPositionPub.set(false);
    m_goToTargetIndexPositionSub = goToTargetIndexPosition.subscribe(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //updateDashboard();
  }

  private void updateDashboard ()
  {
    m_indexPositionPub.set(m_indexEncoder.getPosition());
    m_indexAppliedOutputPub.set(m_indexSparkMax.getAppliedOutput());

    if (m_updateIndexPIDSub.get())
    {
      m_indexConfig.closedLoop.pidf(m_indexPSub.get(), m_indexISub.get(), m_indexDSub.get(), m_indexFFSub.get());

      m_indexSparkMax.configure(m_indexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      
      m_updateIndexPIDPub.set(false);
    }

    if (m_goToTargetIndexPositionSub.get())
    {
      double indexPositionTarget = m_indexTargetRotationsSub.get();
      goToIndexerPosition(indexPositionTarget);
      m_goToTargetIndexPositionPub.set(false);
    }
 
  }

  public void setTargetIndexPosition(CoralIndexTargetPositions targetIndexPosition)
  {
    var targetPosition = CoralIndexPositionValue(targetIndexPosition);
    goToIndexerPosition(targetPosition);
    m_lastTargetPosition = targetPosition;
  }

  private void goToIndexerPosition(double target){
    m_indexController.setReference(target, SparkMax.ControlType.kPosition);
  }

  private static double CoralIndexPositionValue(CoralIndexTargetPositions target)
  {
    switch (target) {
      case Left:
        return kLeftPosition;
      case Right:
        return kRightPosition;
      case Center:
        return kCenterPosition;
      default:
        return 0;
    }
  }

  public boolean isAtIndexTargetPosition()
  {
    return Math.abs(m_lastTargetPosition - m_indexEncoder.getPosition()) < 0.1;
  }

  public void stopAtIndexCurrentPosition()
  {
    goToIndexerPosition(m_indexEncoder.getPosition());
  }

  public void directDrive(double power)
  {
    m_indexSparkMax.set(power);
  }
}
