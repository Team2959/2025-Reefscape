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
    Feed
  };

  public double m_deliveryWaitSeconds = 1;
  public double m_troughAutoDeliveryWaitSeconds = 2;

  private final SparkMax m_coralControlSparkMax = new SparkMax(RobotMap.kCoralDeliveryControlMotor, MotorType.kBrushless);
  private final DigitalInput m_coralDetect = new DigitalInput(RobotMap.kCoralDetectInput);

  private static final double kIntakeSpeed = 0.35;
  private static final double kFeedSpeed = 1.0;
  
  private boolean m_coralPresent = false;

  private final DoubleSubscriber m_velocitySub;
  private final BooleanPublisher m_coralDetectPub;
  private final BooleanSubscriber m_goToTargetVelocitySub;
  private final BooleanPublisher m_goToTargetVelocityPub;

  public CoralDeliverySubsystem() {
     
    var coralControlConfig = new SparkMaxConfig();
    coralControlConfig.idleMode(IdleMode.kBrake);
    coralControlConfig.inverted(true);

    m_coralControlSparkMax.configure(coralControlConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    final String name = "Coral Delivery Subsystem";
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable(name);

    m_coralDetectPub = datatable.getBooleanTopic(name + "/Coral Detect").publish();

    var velocitySub = datatable.getDoubleTopic(name + "/Velocity");
    velocitySub.publish().set(0);
    m_velocitySub = velocitySub.subscribe(0);

    var goToTargetVelocity = datatable.getBooleanTopic(name + "/go To Target Rotations");
    m_goToTargetVelocityPub = goToTargetVelocity.publish();
    m_goToTargetVelocityPub.set(false);
    m_goToTargetVelocitySub = goToTargetVelocity.subscribe(false);
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_coralDetectPub.set(getOpticSensor());

    m_ticks++;
    if (m_ticks % 13 != 11)
        return;

    updateDashboard();
  }

  private void updateDashboard ()
  {
    if(m_goToTargetVelocitySub.get())
    {
      m_coralControlSparkMax.set(m_velocitySub.get());
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
      case Stop:     
      default:
        return 0;
    }
  }

  public void setCoralControlVelocity(CoralControlTargetSpeeds targetSpeed)
  {
    var targetRightEnumSpeed = CoralControlSpeedValue(targetSpeed);
    m_coralControlSparkMax.set(targetRightEnumSpeed);
  }

  public void stopCoralControlMotor ()
  {
    setCoralControlVelocity(CoralControlTargetSpeeds.Stop);
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
