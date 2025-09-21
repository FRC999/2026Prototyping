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
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.lib.VisionHelpers;

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

  @Override
  public void periodic() {
    SwerveDriveState swerveDriveState = RobotContainer.driveSubsystem.getState();
    ChassisSpeeds chassisSpeeds = swerveDriveState.Speeds;

    // One-time IMU mode set: 1 = mirror external yaw into LL IMU (keeps MT2/IMU consistent).
    if (!imuModeSet) {
      for (LLCamera llcamera : LLCamera.values()) {
        LimelightHelpers.SetIMUMode(llcamera.getCameraName(),  LLAprilTagConstants.LLVisionConstants.LL_IMU_MODE);
      }
      imuModeSet = true;
    }

    for (LLCamera llcamera : LLCamera.values()) {
      String cn = llcamera.getCameraName();
      
    

      // Update LLs with current YAW, so they can return correct position for Megatag2
      LimelightHelpers.SetRobotOrientation(cn, 
        swerveDriveState.Pose.getRotation().getDegrees(), 
        Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond), 
        0, 0, 0, 0);
    }
  }
}


