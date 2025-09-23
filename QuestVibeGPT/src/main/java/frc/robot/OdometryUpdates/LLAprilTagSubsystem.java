// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OdometryUpdates;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.PoseEstimate;
import frc.robot.lib.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer;
import frc.robot.OdometryUpdates.LLAprilTagConstants.LLVisionConstants.LLCamera;
import frc.robot.OdometryUpdates.LLAprilTagConstants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.lib.VisionHelpers;
import java.util.function.Supplier;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LLAprilTagSubsystem extends SubsystemBase {
  public static AprilTagFieldLayout fieldLayout;
  
  private boolean imuModeSet = false;

  private double maxBestAmbiguity = 0.5; // Puts pretty high standard on AprilTag position determination

  Map<Pose2d, Integer> allianceTagPoses;

  /** Creates a new LLVisionSubsystem. */
  public LLAprilTagSubsystem() {

    if(!EnabledSubsystems.ll){
      return;
    }

  }

  public Pose2d getRobotAprilTagPose() {
    return null;
  }

  public Pose2d getKnownPose(String poseName) {
    //System.out.println(RobotPoseConstants.visionRobotPoses.keySet());
    if(RobotPoseConstants.visionRobotPoses.containsKey(poseName)){
      return RobotPoseConstants.visionRobotPoses.get(poseName);
    } else {
      return null; 
    }
  }

  public boolean isAprilTagVisible(String cameraName) {
    return LimelightHelpers.getTV(cameraName); 
  }

  public boolean isRedReefTagID(int tag) {
    return ( tag>=6 && tag <=11);
  }
  public boolean isBlueReefTagID(int tag) {
    return ( tag>=17 && tag <=22);
  }
  public boolean isAnyReefTagID(int tag) {
    return isRedReefTagID(tag) || isBlueReefTagID(tag);
  }



  public double getClosestTag(RawFiducial[] rf) { // Closest tag to camera
    double ldr = Double.MAX_VALUE; //lowest distance to robot;
    int ldrid = 0; // id of the closest target
    for (RawFiducial irf : rf) {
      if(irf.distToRobot<ldr) {
        ldr = irf.distToRobot;
        ldrid = irf.id ;
      }
    }
    return ldrid;
  }

  /**
   * Return pose of the alliance apriltag with the IMU closest to the current robot IMU.
   * So, if the bot is oriented generally in the same direction as the tag, that will be the tag returned
   * @return
   */
  public Pose2d getTagPerAllianceAndIMU() {
    Pose2d nearest = new Pose2d();
    double rotationdiff = 180;
    for (Pose2d tagPose : allianceTagPoses.keySet()) {
      double d = Math.abs( RobotContainer.driveSubsystem.getYaw() - tagPose.getRotation().getDegrees() );
      if( d < rotationdiff ) {
        rotationdiff = d;
        nearest = tagPose;
      }
    }
    return nearest;
  }

  public LLCamera[] getListOfApriltagLLCameras() {
    return LLCamera.values();
  }

  public void setLLOrientation(double yaw, double yawrate){
    for (LLCamera llcamera : LLCamera.values()) {
      LimelightHelpers.SetRobotOrientation(llcamera.getCameraName(),  yaw, yawrate,0,0,0,0);
    }
  }

  /**
   * return PoseEstimate from a given camera or NULL if nothing is visible
   * @param cn - camera name
   * @return
   */
  public LimelightHelpers.PoseEstimate getPoseEstimateFromLL(String cn) {
    LimelightHelpers.PoseEstimate pe = RobotContainer.isAllianceRed
      ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cn)
      : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cn);
    if(pe == null){ // If I do not see anything
      return null;
    }
    if(pe.tagCount <= 0){ // less than 1 tag
      return null;
    }

    return pe;
  }

  public LimelightHelpers.PoseEstimate getBestPoseEstimateFromAllLL() {
    LimelightHelpers.PoseEstimate bestPose = null;
    double bestAmbiguity = 99; // Start with a high number

    for (LLCamera llcamera : LLCamera.values()) {
      String cn = llcamera.getCameraName();
      LimelightHelpers.PoseEstimate pe = getPoseEstimateFromLL(cn);
      if (pe != null) {

        if (DebugTelemetrySubsystems.ll) { // Telemetry for LL AT recognition
          SmartDashboard.putNumber("LL " + cn + " PoseEst TagCount", pe.tagCount);
          SmartDashboard.putNumber("LL " + cn + " PoseEst Ambiguity", pe.rawFiducials[0].ambiguity);
        }

        if (pe.rawFiducials[0].ambiguity < bestAmbiguity) {
          bestAmbiguity = pe.rawFiducials[0].ambiguity;
          bestPose = pe;
        }
      } else { // reset telemetry to show that we do not see AT
        if (DebugTelemetrySubsystems.ll) {
          SmartDashboard.putNumber("LL " + cn + " PoseEst TagCount", 0);
          SmartDashboard.putNumber("LL " + cn + " PoseEst Ambiguity", 10);
        }
      }
    }

    // If the best ambiguity is too high, return NULL
    if (bestAmbiguity > maxBestAmbiguity) {
      return null;
    }

    if (DebugTelemetrySubsystems.ll) { // Telemetry for the best Pose selected
      if (bestPose != null) {
        SmartDashboard.putNumber("LL Best PoseEst TagCount", bestPose.tagCount);
        SmartDashboard.putNumber("LL Best PoseEst Ambiguity", bestPose.rawFiducials[0].ambiguity);
      } else {
        SmartDashboard.putNumber("LL Best PoseEst TagCount", 0);
        SmartDashboard.putNumber("LL Best PoseEst Ambiguity", 10);
      }
  }

    return bestPose;
  }

  /**
   * Ordering guard: set orientation, then run the pose query (returns result).
   * 
   * This ensures that setLLOrientation always runs BEFORE the pose query from LL
   */
  public <T> T withOrientation(double yawDeg, double yawRateDegPerSec, Supplier<T> poseQuery) {
    setLLOrientation(yawDeg, yawRateDegPerSec);
    return poseQuery.get();
  }

  /** Ordering guard: set orientation, then run the pose operation (void). */
  public void withOrientation(double yawDeg, double yawRateDegPerSec, Runnable r) {
    setLLOrientation(yawDeg, yawRateDegPerSec);
    r.run();
  }

  /** Raw fiducials for dashboards/debug (kept name/signature). */
  public RawFiducial[] getRawFiducials(String cameraName) {
    return LimelightHelpers.getRawFiducials(cameraName);
  }

  @Override
  public void periodic() {
    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;

    // One-time IMU mode set: 1 = mirror external yaw into LL IMU (keeps MT2/IMU consistent).
    if (!imuModeSet) {
      for (LLCamera llcamera : LLCamera.values()) {
        LimelightHelpers.SetIMUMode(
          llcamera.getCameraName(),
          LLAprilTagConstants.LLVisionConstants.LL_IMU_MODE);
      }
      imuModeSet = true;
    }
  }
}


