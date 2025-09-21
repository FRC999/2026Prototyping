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
import frc.robot.OdometryUpdates.LLAprilTagConstants.LLVisionConstants;
import frc.robot.OdometryUpdates.LLAprilTagConstants.LLVisionConstants.LLCamera;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.QuestHelpers;
import gg.questnav.questnav.PoseFrame;

/**
 * This class is rewritten with Finite State Machine logic (FMS)
 */
public class OdometryUpdatesSubsystem extends SubsystemBase {

  /**
   * LL YAW needs to set as precise as possible before LL can tell you the Pose2D of the bot based on AT.
   * That Yaw is necessary for disambiguation. However, if you rotate the bot after you turn it on, but before
   * it sees any AT, its internal IMU (Pigeon) may not be good enough to track the move.
   * So, the IMU updates necessary for LL disambiguation will be coming from Quest rather that the robot odometry
   * until the SEEDING is done.
   * 
   * The robot posing on the field will be - put it against the game element with exact known angle (e.g reef in 2025)
   * Make sure that angle is known in OdometryConstants. Once the bot is UP, put it to whatever position you need in a starting area,
   * so it sees ATs.
   * Once it sees the ATs, the LL Yaw disambiguation updates will come from the robot odometry.
   * 
   * The robot YAW will be initially set to whatever value is indicated in the OdometryConstants. So, if the
   * ATs do not become visible at the start of auto, or Quest did not come on, the Yaw currently in the odometry will be used. 
   * 
   * FMS states:
   * SEEDING - initial location on the field; QUEST is effectively disabled for odometry updates, but will use its current angle
   * for setting LL Yaw.
   */
  /* ========= State machine for initial alignment ========= */
  private enum VisionState {
    SEEDING, SEEKING_TAGS, CALIBRATED
  }

  private VisionState state = VisionState.SEEDING;

  public boolean gatePassOverride;

  /** Creates a new OdometryUpdatesSubsystem. */
  public OdometryUpdatesSubsystem() {
    gatePassOverride = true;
    
    // The chassis yaw should be initialized regardless of quest or LL being operational
    // TODO: add LED lights and turn it on another color when zero chassis runs
    RobotContainer.driveSubsystem.zeroChassisYaw();
  }

  //Update odometry using Quest
  private void fuseQuestNavAllUnread() {
    // System.out.println(Timer.getFPGATimestamp());
    PoseFrame[] frames = RobotContainer.questNavSubsystem.getAllCurrentPoseframes();
    // System.out.println(Timer.getFPGATimestamp());
    if (frames == null || frames.length == 0) return;
    // System.out.println(Timer.getFPGATimestamp());
    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;


    double speedNow = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Pose2d poseNow = swerveDriveState.Pose;
    boolean gatePassOverrideIntermediate = gatePassOverride;
    for (PoseFrame pf : frames) {
      if (pf == null) continue;

      // Docs: questPose() is the Quest’s Pose2d; convert to robot center with ROBOT_TO_QUEST
      Pose2d questPose = pf.questPose();                         // :contentReference[oaicite:2]{index=2}
      if (questPose == null) continue;
      Pose2d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse()); // :contentReference[oaicite:3]{index=3}

      // Docs: use the data timestamp directly
      Double tMeas = pf.dataTimestamp();                         // seconds (FPGA time) :contentReference[oaicite:4]{index=4}
      double t = (tMeas != null && tMeas > 1.0) ? tMeas : Timer.getFPGATimestamp();

      if (gateMeasurement(robotPose, t, /*strict*/ false, speedNow, poseNow)) {
        Matrix<N3, N1> std = QuestHelpers.questStdDev(speedNow);
        RobotContainer.driveSubsystem.addVisionMeasurement(robotPose, t, QuestNavConstants.QUESTNAV_STD_DEVS);
        System.out.println("T");
        System.out.println(Timer.getFPGATimestamp());
        gatePassOverrideIntermediate = false;
        // System.out.println("TEST");
      } else {
        System.out.println("g: " + robotPose.toString() + " " + t);
      }
    }
    gatePassOverride = gatePassOverrideIntermediate;
  }

  /** Innovation gating with latency compensation via buffered prediction. */
  private boolean gateMeasurement(Pose2d robotPose, double timestamp, boolean strict, double speedNow, Pose2d poseNow) { 
    if (gatePassOverride) {
      return true;
    }
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

  private void fuseLLCamera(LLCamera llcamera) {
    String cn = llcamera.getCameraName();
    if (cn == null || cn.isBlank()) return;

    LimelightHelpers.PoseEstimate pe = RobotContainer.isAllianceRed
        ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cn)
        : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cn);
    if (pe == null) return;

    int tagCount = pe.tagCount;
    if (tagCount <= 0) return;

    double ambiguity = LimelightHelpers.clamp(pe.rawFiducials[0].ambiguity, 0.0, 1.0); // if your Helpers expose it; else set from MT2 metrics
    
    if ((tagCount == 1 && ambiguity > LLVisionConstants.kMaxSingleTagAmbiguity) 
        || (pe.rawFiducials[0].distToCamera > LLVisionConstants.kMaxCameraToTargetDistance)) 
      return;

    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;

    double speedNow = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Pose2d poseNow = swerveDriveState.Pose;

    Pose2d robotPose = pe.pose; // robot-in-field (requires correct cam extrinsics in LL UI)
    double timestampLL = (pe.timestampSeconds > 1.0) ? pe.timestampSeconds
      : Timer.getFPGATimestamp() - pe.latency;


    if (!gatePassOverride && !gateMeasurement(robotPose, timestampLL, /*strict*/ true, speedNow, poseNow)) return;

    if(!RobotContainer.questNavSubsystem.isInitialPoseSet() && tagCount > 0 && ambiguity < LLVisionConstants.kMaxQuestCalibrationAmbiguity ) {
      calibrateQuestFromLL(robotPose);
      RobotContainer.driveSubsystem.resetCTREPose(robotPose);
      gatePassOverride = false;
      RobotContainer.questNavSubsystem.setInitialPoseSet(true);
    }

    Matrix<N3, N1> std = LimelightHelpers.llStdDev(pe.avgTagDist, tagCount, ambiguity);
    RobotContainer.driveSubsystem.addVisionMeasurement(robotPose, timestampLL, std);
  }

  private void calibrateQuestFromLL(Pose2d robotPose) {
    RobotContainer.questNavSubsystem.resetQuestOdometry(robotPose);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(Timer.getFPGATimestamp());
    if(RobotContainer.questNavSubsystem.isInitialPoseSet()){ //Only use quest if initial quest pose is set
      //QuestNav: process ALL unread PoseFrames (high trust)
      fuseQuestNavAllUnread();
    }

    if(!RobotContainer.questNavSubsystem.isInitialPoseSet()) {

      for (LLCamera llcamera: RobotContainer.llAprilTagSubsystem.getListOfApriltagLLCameras()) {
        fuseLLCamera(llcamera);
      }
      
    }
  }
}
