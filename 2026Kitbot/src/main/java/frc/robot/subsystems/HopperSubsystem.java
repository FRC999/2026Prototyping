// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class HopperSubsystem extends SubsystemBase {
  WPI_VictorSPX hopperMotor = new WPI_VictorSPX(IntakeConstants.launcherMotorID);
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    configureMotors();
  }

  public void configureMotors(){
    VictorSPXConfiguration configs = new VictorSPXConfiguration();
    hopperMotor.configAllSettings(configs);
  }

  public void setHopperMotor(double speed){
    hopperMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    hopperMotor.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
