// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OdometryUpdates;

import java.awt.Robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.QuestHelpers;
import gg.questnav.questnav.PoseFrame;

public class OdometryUpdatesSubsystem extends SubsystemBase {


  /** Creates a new OdometryUpdatesSubsystem. */
  public OdometryUpdatesSubsystem() {

  }

  //Update odometry using Quest
  private void fuseQuestNavAllUnread() {
    PoseFrame[] frames = RobotContainer.questNavSubsystem.getAllUnreadPoseFrames();
    if (frames == null || frames.length == 0) return;

    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;


    double speedNow = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Pose2d poseNow = swerveDriveState.Pose;

    for (PoseFrame pf : frames) {
      if (pf == null) continue;

      // Docs: questPose() is the Quest’s Pose2d; convert to robot center with ROBOT_TO_QUEST
      Pose2d questPose = pf.questPose();                         // :contentReference[oaicite:2]{index=2}
      if (questPose == null) continue;
      Pose2d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse()); // :contentReference[oaicite:3]{index=3}

      // Docs: use the data timestamp directly
      Double tMeas = pf.dataTimestamp();                         // seconds (FPGA time) :contentReference[oaicite:4]{index=4}
      double t = (tMeas != null && tMeas > 1.0) ? tMeas : Timer.getFPGATimestamp();

      if (!gateMeasurement(robotPose, t, /*strict*/ false, speedNow, poseNow)) continue;

      Matrix<N3, N1> std = QuestHelpers.questStdDev(speedNow);
      RobotContainer.driveSubsystem.addVisionMeasurement(robotPose, t, std);
    }
  }

  /** Innovation gating with latency compensation via buffered prediction. */
  private boolean gateMeasurement(Pose2d robotPose, double timestamp, boolean strict, double speedNow, Pose2d poseNow) { 
    //Either pose from driveSubsystem at timestamp or current pose of bot
    Pose2d chassisPoseAtTimestamp = RobotContainer.driveSubsystem.getSample(timestamp).orElse(poseNow);

    var twist = chassisPoseAtTimestamp.minus(robotPose); // log(SE2)
    double transErr = Math.hypot(twist.getX(), twist.getY());
    double rotErr = Math.abs(twist.getRotation().getRadians());

    //Tolerances for translation and rotation that determine validity of vision Pose
    final double transGate = (strict ? 0.6 : 0.8) + 0.5 * speedNow;  // meters
    final double rotGate   = Units.degreesToRadians(strict ? 15.0 : 20.0);

    return transErr <= transGate && rotErr <= rotGate;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(RobotContainer.questNavSubsystem.isInitialPoseSet()){ //Only use quest if initial quest pose is set
      //QuestNav: process ALL unread PoseFrames (high trust)
      fuseQuestNavAllUnread();
    }
  }
}
