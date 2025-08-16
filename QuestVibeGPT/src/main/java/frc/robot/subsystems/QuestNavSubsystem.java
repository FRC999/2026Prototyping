// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav;
  Transform2d ROBOT_TO_QUEST = new Transform2d(-0.4, -0.3, Rotation2d.k180deg);
  Pose2d robotPose = new Pose2d(0, 0, Rotation2d.kZero);

  PoseFrame[] poseFrames;

  /** Creates a new QuestNavSubsystem. */
  public QuestNavSubsystem() {
    questNav = new QuestNav();

    resetToZeroPose();

  }

  public void resetToZeroPose() {
    Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
    System.out.println("****QRobot reset to zero pose: " + questPose.toString());
  }

  public void resetQuestIMUToAngle(double angle) {
    // questNav.setPose(new Pose2d(getQuestPose().getTranslation(),
    // ROBOT_TO_QUEST.getRotation()
    // .rotateBy(Rotation2d.fromDegrees(angle))));
    System.out.println("Quest Robot Pose: " + 
        java.util.Objects.requireNonNullElse(getQuestRobotPose(),"").toString());
    System.out.println("QAngle: " + angle);
    System.out.println("New QAngle: " + (Rotation2d.fromDegrees(angle).
        minus(getQuestRobotPose().getRotation())).getDegrees());
    
    Pose2d newRobotPose = new Pose2d(getQuestRobotPose().getTranslation(), Rotation2d.fromDegrees(angle));
    System.out.println(newRobotPose.toString());
    questNav.setPose(newRobotPose.transformBy(ROBOT_TO_QUEST));
  }

  /**
   * Gets the yaw of the QuestNav (Z axis rotation) (yaw is the direction that the
   * questnav is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getTrueYaw() {
    return getQuestPose().getRotation().getDegrees(); // With True QuestNav Pose this method returns values in degrees
  }

  public double getQuestRobotYaw() {
    return getQuestRobotPose().getRotation().getDegrees();
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double previousYaw = getQuestRobotYaw();
    // System.out.println("Old Yaw: " + previousYaw);
    if (RobotContainer.isAllianceRed
        && RobotContainer.isReversingControllerAndIMUForRed) {
      // System.out.println("Yaw 180 " + RobotContainer.isAllianceRed);
      resetQuestIMUToAngle(180);

    } else {
      //System.out.println("Yaw NOT 180 " + RobotContainer.isAllianceRed);
      resetQuestIMUToAngle(0);
    }
    System.out.println("New Yaw: " + getQuestRobotYaw());
    return previousYaw;
  }

  public Pose2d getQuestRobotPose() {

    return (poseFrames != null && poseFrames.length > 0) ?
      poseFrames[poseFrames.length - 1].questPose()
        .transformBy(ROBOT_TO_QUEST.inverse()) :
      null;
    
    // return qPose;
  }

  public double getQTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ?
      poseFrames[poseFrames.length - 1].dataTimestamp() :
      0;
  }

  public double getQAppTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ?
        poseFrames[poseFrames.length - 1].appTimestamp() :
        0;
  }

  public Pose2d getQuestPose() {
    return (poseFrames != null && poseFrames.length > 0) ?
        poseFrames[poseFrames.length - 1].questPose() :
        null;
  }

  public void resetQuestOdometry(Pose2d robotPose) {

    // Transform by the offset to get the Quest pose
    Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

    // Send the reset operation
    questNav.setPose(questPose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("qTranformedPose: ", getQuestRobotPose().toString());
    SmartDashboard.putString("qTruePose: ", getQuestPose().toString());
    SmartDashboard.putNumber("TimeStamp: ", getQTimeStamp());
    SmartDashboard.putNumber("TimeStampA: ", getQAppTimeStamp());
    SmartDashboard.putNumber("TimeStampFPGS: ", Utils.fpgaToCurrentTime(getQTimeStamp()));


    //update pose Frames
    poseFrames = questNav.getAllUnreadPoseFrames();
    // Display number of frames provided
    SmartDashboard.putNumber("qFrames", poseFrames.length);
  }
}
