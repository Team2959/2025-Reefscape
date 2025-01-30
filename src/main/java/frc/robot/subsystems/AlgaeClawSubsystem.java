// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeClawSubsystem extends SubsystemBase {
  /** Creates a new AlgaeClawSubsystem. */
  private final SparkMax m_clawShootSparkMax = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax m_clawIntakeSparkMax = new SparkMax(8, MotorType.kBrushless);
  private SparkRelativeEncoder m_clawShootEncoder;
  private SparkRelativeEncoder m_clawIntakeEncoder;
  private final SparkMaxConfig m_clawShootConfig;
  private final SparkMaxConfig m_clawIntakeConfig;

  public AlgaeClawSubsystem() {
    m_clawShootConfig = new SparkMaxConfig();
    m_clawShootConfig.idleMode(IdleMode.kBrake);

    m_clawIntakeConfig = new SparkMaxConfig();
    m_clawIntakeConfig.idleMode(IdleMode.kBrake);

    m_clawShootSparkMax.configure(m_clawShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_clawIntakeSparkMax.configure(m_clawIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_clawShootEncoder = (SparkRelativeEncoder) m_clawShootSparkMax.getEncoder(); 
    m_clawIntakeEncoder = (SparkRelativeEncoder) m_clawIntakeSparkMax.getEncoder(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
