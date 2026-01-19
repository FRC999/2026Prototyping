// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.intakeMotorID);
  WPI_VictorSPX launcherMotor = new WPI_VictorSPX(IntakeConstants.launcherMotorID);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    configureMotors();
  }

  public void configureMotors(){
    VictorSPXConfiguration configs = new VictorSPXConfiguration();
    intakeMotor.configAllSettings(configs);
    launcherMotor.configAllSettings(configs);
  }

  public void setIntakeMotor(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public void setLauncherMotor(double speed){
    launcherMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    intakeMotor.set(0);
    launcherMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
