// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;

import com.ctre.phoenix6.Utils;

public class QuestNavSubsystem extends SubsystemBase {
  

  QuestNav questNav = new QuestNav();
  Transform2d ROBOT_TO_QUEST = new Transform2d();

  /** Creates a new QuestNavSubsystem. */
  public QuestNavSubsystem() {}

  public Pose2d getQuestPose() {
    Pose2d questPose = questNav.getPose();
    Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
    return robotPose;
  }

  public double getQuestTimestamp() {
    double timestamp = questNav.getDataTimestamp();
    return timestamp;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (questNav.isConnected() && questNav.isTracking()) {
      SmartDashboard.putString("QuestPose:", getQuestPose().toString());
      double timestamp = getQuestTimestamp();
      SmartDashboard.putNumber("QuestTimestamp", timestamp);
      SmartDashboard.putNumber("QuestTimestamp fga", Utils.fpgaToCurrentTime(timestamp));
      SmartDashboard.putNumber("Phoenix Current Time", Utils.getCurrentTimeSeconds());
    }
  }
}
