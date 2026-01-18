// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  //Put in the ID's when installed in the robot
  WPI_VictorSPX leadLeftMotor = new WPI_VictorSPX(DriveConstants.leftLeadCANID);
  WPI_VictorSPX followLeftMotor = new WPI_VictorSPX(DriveConstants.leftFollowCANID);
  WPI_VictorSPX leadRightMotor = new WPI_VictorSPX(DriveConstants.rightLeadCANID);
  WPI_VictorSPX followRightMotor = new WPI_VictorSPX(DriveConstants.rightFollowCANID);

  DifferentialDrive drive = new DifferentialDrive(
      leadLeftMotor, leadRightMotor);

  public DriveSubsystem() {
    configureMotors();
  }  

  private void configureMotors(){
    VictorSPXConfiguration configs = new VictorSPXConfiguration();
    leadLeftMotor.configAllSettings(configs);
    followLeftMotor.configAllSettings(configs);
    leadRightMotor.configAllSettings(configs);
    followRightMotor.configAllSettings(configs);

    leadLeftMotor.setInverted(DriveConstants.leftLeadInverted);
    followLeftMotor.setInverted(DriveConstants.leftLeadInverted);
    leadRightMotor.setInverted(DriveConstants.leftLeadInverted);
    followRightMotor.setInverted(DriveConstants.leftLeadInverted);
        
    followLeftMotor.follow(leadLeftMotor);
    followRightMotor.follow(leadRightMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveArcade(double xVel, double zRot){
    drive.arcadeDrive(xVel, zRot);
  }
}
