// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.ElasticHelpers;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */

  

  
  public SmartDashboardSubsystem() {}


  public void updateLLTelemetry() {
    SmartDashboard.putString("LL4-Visible", ElasticHelpers.LLAnyVisibleColors(RobotContainer.llAprilTagSubsystem.isAprilTagVisibleAny()));
  }

  public void SystemsCheckTelemetry() {
    SmartDashboard.putNumber("Battery-Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putString("Alliance-Side", ElasticHelpers.getAllianceSide());
    SmartDashboard.putString("Auto-Selected", ElasticHelpers.getAutoSelectedColor());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void TeleopTelemetry() {
    SmartDashboard.putData("Field", ElasticHelpers.getRobotonfield());
    SmartDashboard.putData("Auto Field", ElasticHelpers.getAutoDisplayField());
    SmartDashboard.putString("Lock in to End Game", ElasticHelpers.shouldEndGameColor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
    ElasticHelpers.updateRobotPose(robotPose);

    RobotContainer.updateAutoPathPreview();

    updateLLTelemetry();
    SystemsCheckTelemetry();
    TeleopTelemetry();
  }
}