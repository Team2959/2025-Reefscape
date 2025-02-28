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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralDeliverySubsystem extends SubsystemBase {
  /** Creates a new CoralDeliverySubsystem. */
  public enum CoralIndexTargetPositions 
  {
    Left,
    Right,
    Center
  };

  public enum CoralControlTargetSpeeds 
  {
    Stop,
    Intake,
    Feed,
    L1FastSpeed,  // Fast/slow are for ensuring the coral spins either left or right out of the feeder, depending on desired delivery
    L1SlowSpeed   // If spit left, right side motor fast, left side motor slow
  };
  public static double DeliveryWaitSeconds = 1;

  private final SparkMax m_indexSparkMax = new SparkMax(RobotMap.kCoralDeliveryIndexMotor, MotorType.kBrushed);
  private final SparkMax m_rightCoralControlSparkMax = new SparkMax(RobotMap.kCoralDeliveryRightCoralControlMotor, MotorType.kBrushless);
  private final SparkMax m_leftCoralControlSparkMax = new SparkMax(RobotMap.kCoralDeliveryLeftCoralControlMotor, MotorType.kBrushless);
  private final SparkMaxAlternateEncoder m_indexEncoder;
  private final SparkMaxConfig m_indexConfig;
  private final SparkClosedLoopController m_indexController;
  private final DigitalInput m_coralDetect = new DigitalInput(RobotMap.kCoralDetectInput);

  private final double kIndexP = 1.8;
  private final double kIndexI = 0.0;
  private final double kIndexD = 0;
  private final double kIndexFf = 0.0;
  private double m_lastTargetPosition;

  private final DoublePublisher m_indexPositionPub;
  private final DoublePublisher m_indexAppliedOutputPub;
  private final DoubleSubscriber m_rightVelocitySub;
  private final DoubleSubscriber m_leftVelocitySub;
  private final BooleanPublisher m_coralDetectPub;
  private final BooleanSubscriber m_goToTargetVelocitySub;
  private final BooleanPublisher m_goToTargetVelocityPub;
  private final DoubleSubscriber m_indexPSub;
  private final DoubleSubscriber m_indexISub;
  private final DoubleSubscriber m_indexDSub;
  private final DoubleSubscriber m_indexFFSub;
  private final BooleanPublisher m_updateIndexPIDPub;
  private final BooleanSubscriber m_updateIndexPIDSub;
  private final DoubleSubscriber m_indexTargetRotationsSub;
  private final BooleanSubscriber m_goToTargetIndexPositionSub;
  private final BooleanPublisher m_goToTargetIndexPositionPub;

  public CoralDeliverySubsystem() {
 
    m_indexConfig = new SparkMaxConfig();
    m_indexConfig.idleMode(IdleMode.kCoast);
    m_indexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pidf(kIndexP, kIndexI, kIndexD, kIndexFf);
    var alternateEncoderConfig = new AlternateEncoderConfig();
    alternateEncoderConfig.setSparkMaxDataPortConfig();
    m_indexConfig.apply(alternateEncoderConfig);
    
    var rightCoralControlConfig = new SparkMaxConfig();
    rightCoralControlConfig.idleMode(IdleMode.kBrake);

    var leftCoralControlConfig = new SparkMaxConfig();
    leftCoralControlConfig.idleMode(IdleMode.kBrake);
    leftCoralControlConfig.inverted(true);

    m_indexSparkMax.configure(m_indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightCoralControlSparkMax.configure(rightCoralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftCoralControlSparkMax.configure(leftCoralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_indexEncoder = (SparkMaxAlternateEncoder)m_indexSparkMax.getAlternateEncoder();
    m_indexController = m_indexSparkMax.getClosedLoopController();

    final String name = "Coral Delivery Subsystem";
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable(name);

    m_indexPositionPub = datatable.getDoubleTopic(name + "/Index Position").publish();
    m_indexAppliedOutputPub = datatable.getDoubleTopic(name + "/Index Applied Output").publish();
    m_coralDetectPub = datatable.getBooleanTopic(name + "/Coral Detect").publish();

    var leftVelocitySub = datatable.getDoubleTopic(name + "/Left Velocity");
    leftVelocitySub.publish().set(0);
    m_leftVelocitySub = leftVelocitySub.subscribe(0);

    var rightVelocitySub = datatable.getDoubleTopic(name + "/Right Velocity");
    rightVelocitySub.publish().set(0);
    m_rightVelocitySub = rightVelocitySub.subscribe(0);

    var goToTargetVelocity = datatable.getBooleanTopic(name + "/go To Target Rotations");
    m_goToTargetVelocityPub = goToTargetVelocity.publish();
    m_goToTargetVelocityPub.set(false);
    m_goToTargetVelocitySub = goToTargetVelocity.subscribe(false);

    var indexPSub = datatable.getDoubleTopic(name + "/index P");
    indexPSub.publish().set(kIndexP);
    m_indexPSub = indexPSub.subscribe(kIndexP);

    var indexISub = datatable.getDoubleTopic(name + "/index I");
    indexISub.publish().set(kIndexI);
    m_indexISub = indexISub.subscribe(kIndexI);

    var indexDSub = datatable.getDoubleTopic(name + "/index D");
    indexDSub.publish().set(kIndexD);
    m_indexDSub = indexDSub.subscribe(kIndexD);

    var indexFFSub = datatable.getDoubleTopic(name + "/index FF");
    indexFFSub.publish().set(kIndexFf);
    m_indexFFSub = indexFFSub.subscribe(kIndexFf);

    var updateIndexPIDPub = datatable.getBooleanTopic(name + "/update Index PID");
    m_updateIndexPIDPub = updateIndexPIDPub.publish();
    m_updateIndexPIDPub.set(false);
    m_updateIndexPIDSub = updateIndexPIDPub.subscribe(false);

    var indexTargetPositionSub = datatable.getDoubleTopic(name + "/Index Target Position");
    indexTargetPositionSub.publish().set(0);
    m_indexTargetRotationsSub = indexTargetPositionSub.subscribe(0);

    var goToTargetIndexPosition = datatable.getBooleanTopic(name + "/Go To Target Index Position");
    m_goToTargetIndexPositionPub = goToTargetIndexPosition.publish();
    m_goToTargetIndexPositionPub.set(false);
    m_goToTargetIndexPositionSub = goToTargetIndexPosition.subscribe(false);
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ticks++;
    if (m_ticks % 13 != 11)
        return;

    updateDashboard();
  }

  private void updateDashboard ()
  {
    m_coralDetectPub.set(m_coralDetect.get());
    m_indexPositionPub.set(m_indexEncoder.getPosition());
    m_indexAppliedOutputPub.set(m_indexSparkMax.getAppliedOutput());

    if(m_goToTargetVelocitySub.get())
    {
      m_leftCoralControlSparkMax.set(m_leftVelocitySub.get());
      m_rightCoralControlSparkMax.set(m_rightVelocitySub.get());
      m_goToTargetVelocityPub.set(false);
    }

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
        return -7.37;
      case Right:
        return -0.1;
      case Center:
        return -3.7;
      default:
        return 0;
    }
  }
  
  private static double CoralControlSpeedValue (CoralControlTargetSpeeds target)
  {
    switch (target) {
      case Intake:
        return 0.5;
      case Feed:
        return 1.0;
      case L1FastSpeed:
        return 0.7;
      case L1SlowSpeed:
        return 0.1;  
      case Stop:     
      default:
        return 0;
    }
  }
  
  public boolean isAtIndexTargetPosition()
  {
    return Math.abs(m_lastTargetPosition - m_indexEncoder.getPosition()) < 0.05;
  }

  public void stopAtIndexCurrentPosition()
  {
    goToIndexerPosition(m_indexEncoder.getPosition());
  }

  public void setLeftCoralControlVelocity(CoralControlTargetSpeeds targetSpeed)
  {
    var targetLeftEnumSpeed = CoralControlSpeedValue(targetSpeed);
    m_leftCoralControlSparkMax.set(targetLeftEnumSpeed);
  }

  public void setRightCoralControlVelocity(CoralControlTargetSpeeds targetSpeed)
  {
    var targetRightEnumSpeed = CoralControlSpeedValue(targetSpeed);
    m_rightCoralControlSparkMax.set(targetRightEnumSpeed);
  }

  public void stopRightCoralControlMotor ()
  {
    setRightCoralControlVelocity(CoralControlTargetSpeeds.Stop);
  }

  public void stopLeftCoralControlMotor ()
  {
    setLeftCoralControlVelocity(CoralControlTargetSpeeds.Stop);
  }

  public boolean getOpticSensor()
  {
    return m_coralDetect.get();
  }

  public void directDrive(double power)
  {
    m_indexSparkMax.set(power);
  }
}
