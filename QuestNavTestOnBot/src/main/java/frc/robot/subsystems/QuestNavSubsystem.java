// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav;
  Transform2d ROBOT_TO_QUEST = new Transform2d(-0.4, -0.3, Rotation2d.k180deg);



  /** Creates a new QuestNavSubsystem. */
  public QuestNavSubsystem() {
    questNav = new QuestNav();

  }

  public Pose2d getNavPose() {
    Pose2d qPose = questNav.getPose();
    //Pose2d robotPose = qPose.transformBy(ROBOT_TO_QUEST.inverse());
    //return robotPose;
    return qPose;  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("qPose: ", getNavPose().toString());
  }
}
