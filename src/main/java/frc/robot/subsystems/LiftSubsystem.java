// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
  private final SparkMax m_lift = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax m_liftFollower = new SparkMax(6, MotorType.kBrushless);

  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
     
    m_lift.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(5, true);
    m_liftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void directDrive(double power) 
  {
    m_lift.set(power);
  }
}
