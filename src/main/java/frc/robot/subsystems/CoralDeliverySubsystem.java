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
import com.revrobotics.spark.config.MAXMotionConfig;
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

  private final SparkMax m_indexSparkMax = new SparkMax(RobotMap.kCoralDeliveryIndexMotor, MotorType.kBrushless);
  //private final SparkMax m_rightCoralControlSparkMax = new SparkMax(RobotMap.kCoralDeliveryRightCoralControlMotor, MotorType.kBrushless);
  //private final SparkMax m_leftCoralControlSparkMax = new SparkMax(RobotMap.kCoralDeliveryLeftCoralControlMotor, MotorType.kBrushless);
  private SparkRelativeEncoder m_indexEncoder;
  private final SparkMaxConfig m_indexConfig;
  private SparkClosedLoopController m_indexController;
  private final DigitalInput m_coralDetect = new DigitalInput(RobotMap.kCoralDetectInput);


  private final double kIndexP = 0.008;
  private final double kIndexI = 0.000001;
  private final double kIndexD = 0;
  private final double kIndexFf = 0.0008;
  private double m_lastTargetPosition;

  private final DoublePublisher m_indexPositionPub;
  private final DoublePublisher m_indexVelocityPub;
  private final DoublePublisher m_indexAppliedOutputPub;
  private final DoubleSubscriber m_rightVelocitySub;
  private final DoubleSubscriber m_leftVelocitySub;
  private final BooleanPublisher m_coralDetectPub;
  private final BooleanSubscriber m_goToTargetVelocitySub;
  private final BooleanPublisher m_goToTargetVelocityPub;
  private final DoubleSubscriber m_indexPSub;
  private final DoubleSubscriber m_indexISub;
  private final DoubleSubscriber m_indexDSub;
  private final BooleanPublisher m_updateIndexPIDPub;
  private final BooleanSubscriber m_updateIndexPIDSub;
  private final DoubleSubscriber m_indexTargetRotationsSub;
  private final BooleanSubscriber m_goToTargetIndexPositionSub;
  private final BooleanPublisher m_goToTargetIndexPositionPub;

  public CoralDeliverySubsystem() {
 
    m_indexConfig = new SparkMaxConfig();
    m_indexConfig.idleMode(IdleMode.kCoast);
    m_indexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kIndexP, kIndexI, kIndexD, kIndexFf);
    var maxMotion = new MAXMotionConfig().maxVelocity(4000).maxAcceleration(3000);
    m_indexConfig.closedLoop.apply(maxMotion);
    
    var coralControlConfig = new SparkMaxConfig();
    coralControlConfig.idleMode(IdleMode.kBrake);

    m_indexSparkMax.configure(m_indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //m_rightCoralControlSparkMax.configure(coralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //m_leftCoralControlSparkMax.configure(coralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_indexEncoder = (SparkRelativeEncoder) m_indexSparkMax.getEncoder(); 

    m_indexController = m_indexSparkMax.getClosedLoopController();

    final String name = "Coral Delivery Subsystem";
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable(name);

    m_indexPositionPub = datatable.getDoubleTopic(name + "/Index Position").publish();
    m_indexVelocityPub = datatable.getDoubleTopic(name + "/Index Velocity").publish();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateDashboard();
  }

  private void updateDashboard ()
  {
    m_coralDetectPub.set(m_coralDetect.get());
    m_indexPositionPub.set(m_indexEncoder.getPosition());
    m_indexVelocityPub.set(m_indexEncoder.getVelocity());
    m_indexAppliedOutputPub.set(m_indexSparkMax.getAppliedOutput());

    double leftVelocityTarget = m_leftVelocitySub.get();
    double rightVelocityTarget = m_rightVelocitySub.get();
    double indexPositionTarget = m_indexTargetRotationsSub.get();

    if(m_goToTargetVelocitySub.get())
    {
      //m_leftCoralControlSparkMax.set(leftVelocityTarget);
      //m_rightCoralControlSparkMax.set(rightVelocityTarget);
      m_goToTargetVelocityPub.set(false);
    }

    if (m_updateIndexPIDSub.get())
    {
      m_indexConfig.closedLoop.pid(m_indexPSub.get(), m_indexISub.get(), m_indexDSub.get());
      m_indexSparkMax.configure(m_indexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      
      m_updateIndexPIDPub.set(false);
    }

    if (m_goToTargetIndexPositionSub.get())
    {
      m_indexController.setReference(indexPositionTarget, SparkMax.ControlType.kPosition);
      m_goToTargetIndexPositionPub.set(false);
    }
  }

  public double setTargetIndexPosition (CoralIndexTargetPositions targetIndexPosition)
  {
    var targetPosition = CoralIndexPositionValue(targetIndexPosition);
    m_indexController.setReference(targetPosition, SparkMax.ControlType.kPosition);
    m_lastTargetPosition = targetPosition;
    return m_lastTargetPosition;
  }

  private static int CoralIndexPositionValue(CoralIndexTargetPositions target)
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
  
  private static double CoralControlSpeedValue (CoralControlTargetSpeeds target)
  {
    switch (target) {
      case Intake:
        return 0.5; //random numbers
      case Feed:
        return 0.5;
      case L1FastSpeed:
        return 0.75;
      case L1SlowSpeed:
        return 0.25;  
      case Stop:     
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
    m_indexController.setReference(m_indexEncoder.getPosition(), SparkMax.ControlType.kMAXMotionPositionControl);
  }

  public void setLeftCoralControlVelocity(CoralControlTargetSpeeds targetSpeed)
  {
    var targetLeftEnumSpeed = CoralControlSpeedValue(targetSpeed);
    //m_leftCoralControlSparkMax.set(-targetLeftEnumSpeed);
  }

  public void setRightCoralControlVelocity(CoralControlTargetSpeeds targetSpeed)
  {
    var targetRightEnumSpeed = CoralControlSpeedValue(targetSpeed);
    //m_rightCoralControlSparkMax.set(targetRightEnumSpeed);
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
