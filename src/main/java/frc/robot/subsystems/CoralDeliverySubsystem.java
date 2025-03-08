// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralDeliverySubsystem extends SubsystemBase {
  /** Creates a new CoralDeliverySubsystem. */

  public enum CoralControlTargetSpeeds 
  {
    Stop,
    Intake,
    Feed,
    FeedL4,
    L1FastSpeed,  // Fast/slow are for ensuring the coral spins either left or right out of the feeder, depending on desired delivery
    L1SlowSpeed,   // If spit left, right side motor fast, left side motor slow
    L1AutoFastSpeed
  };

  public double m_deliveryWaitSeconds = 1;

  private final SparkMax m_rightCoralControlSparkMax = new SparkMax(RobotMap.kCoralDeliveryRightCoralControlMotor, MotorType.kBrushless);
  private final SparkMax m_leftCoralControlSparkMax = new SparkMax(RobotMap.kCoralDeliveryLeftCoralControlMotor, MotorType.kBrushless);
  private final DigitalInput m_coralDetect = new DigitalInput(RobotMap.kCoralDetectInput);

  private static final double kIntakeSpeed = 0.35;
  private static final double kFeedSpeed = 1.0;
  private static final double kL1FastSpeed = 0.7;
  private static final double kL1AutoFastSpeed = 0.4;
  private static final double kL1SlowSpeed = 0.1;
  private static final double kL4FeedSpeed = 1.0;
  
  private boolean m_coralPresent = false;

  private final DoubleSubscriber m_rightVelocitySub;
  private final DoubleSubscriber m_leftVelocitySub;
  private final BooleanPublisher m_coralDetectPub;
  private final BooleanSubscriber m_goToTargetVelocitySub;
  private final BooleanPublisher m_goToTargetVelocityPub;

  public CoralDeliverySubsystem() {
     
    var rightCoralControlConfig = new SparkMaxConfig();
    rightCoralControlConfig.idleMode(IdleMode.kBrake);

    var leftCoralControlConfig = new SparkMaxConfig();
    leftCoralControlConfig.idleMode(IdleMode.kBrake);
    leftCoralControlConfig.inverted(true);

    m_rightCoralControlSparkMax.configure(rightCoralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftCoralControlSparkMax.configure(leftCoralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    final String name = "Coral Delivery Subsystem";
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable(name);

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
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // m_coralDetectPub.set(getOpticSensor());

   // m_ticks++;
  //  if (m_ticks % 13 != 11)
   //     return;

    //updateDashboard();
  }

  private void updateDashboard ()
  {
    if(m_goToTargetVelocitySub.get())
    {
      m_leftCoralControlSparkMax.set(m_leftVelocitySub.get());
      m_rightCoralControlSparkMax.set(m_rightVelocitySub.get());
      m_goToTargetVelocityPub.set(false);
    }
 }
  
  private static double CoralControlSpeedValue (CoralControlTargetSpeeds target)
  {
    switch (target) {
      case Intake:
        return kIntakeSpeed;
      case Feed:
        return kFeedSpeed;
      case FeedL4:
        return kL4FeedSpeed;
      case L1FastSpeed:
        return kL1FastSpeed;
      case L1SlowSpeed:
        return kL1SlowSpeed;  
      case L1AutoFastSpeed:
        return kL1AutoFastSpeed;
      case Stop:     
      default:
        return 0;
    }
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
    return !m_coralDetect.get();
  }

  public void setCoralPresent(boolean coralPresent)
  {
    m_coralPresent = coralPresent;
  }

  public boolean isCoralInPosition ()
  {
    return m_coralPresent && !getOpticSensor();
  }
}
