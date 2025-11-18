// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
public class MotorControl extends SubsystemBase {
 private SparkMax m_leadmotor;
 private SparkMax m_followMotor;
 
 public MotorControl() {
    m_leadmotor = new SparkMax(54 , Motortype.kBrushless);
    m_followMotor = new SparkMax(57, Motortype.kBrushless);
    SparkMaxConfig leadConfig = new SparkMaxConfig();
    SparkMaxConfig followConfig = new SparkMaxConfig();
    leadConfig.idleMode(IdleMode.kBrake);   
    leadConfig.smartCurrentLimit(50);
    followConfig.apply(leadConfig);
    followConfig.inverted(true);
    m_leadmotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(followConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
