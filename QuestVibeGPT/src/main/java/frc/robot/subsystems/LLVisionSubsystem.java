// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.PoseEstimate;
import frc.robot.lib.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.LLVisionConstants;
import frc.robot.Constants.LLVisionConstants.LLCamera;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.lib.VisionHelpers;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LLVisionSubsystem extends SubsystemBase {
  public static AprilTagFieldLayout fieldLayout;

  private Pose2d bestPose;
  private Pose2d bestPoseLL4;
  private double bestPoseTimestamp;
  private double bestPoseTimestampLL4;
  private boolean bestVisible;
  private boolean bestVisibleLL4;

  private double maxBestAmbiguity = 0.5; // Puts pretty high standard on AprilTag position determination

  Map<Pose2d, Integer> allianceTagPoses;

  /** Creates a new LLVisionSubsystem. */
  public LLVisionSubsystem() {

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

  public boolean isAprilTagVisibleBySomeCamera() {
    return bestVisible; 
  }

  public boolean isAprilTagVisibleByLL4() {
    return bestVisibleLL4; 
  }

  public Pose2d getBestPoseAllCameras() {
    return bestPose;
  }

  public Pose2d getBestPoseLL4s() {
    return bestPoseLL4;
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

  // public void ToggleBackLLMode() {
  //   if (backLLModeAprilTag) { // Switch to driver camera
  //     LimelightHelpers.setPipelineIndex(LLVisionConstants.LLCamera.LLBACK.getCameraName(),1);
  //   } else { // Switch to apriltags
  //     LimelightHelpers.setPipelineIndex(LLVisionConstants.LLCamera.LLBACK.getCameraName(),0);
  //   }
  // }

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (fieldLayout != null) {
    //   // Example: Get the pose of AprilTag with ID 1
    //   int tagId = 1;
    //   Pose3d tagPose = fieldLayout.getTagPose(tagId).orElse(null);

    //   if (tagPose != null) {
    //     System.out.println("AprilTag " + tagId + " Pose: " + tagPose);
    //   } else {
    //     System.out.println("AprilTag " + tagId + " not found in the field layout.");
    //   }
    // }

    boolean tvisible = false; // do we see AT this cycle anywhere?
    boolean tvisibleLL4 = false; // do we see AT this cycle anywhere?


    for (LLCamera llcamera : LLCamera.values()) {
      String cn = llcamera.getCameraName();
      
      boolean ll4 = (cn.length() < 14); // check is this LL4?

      double yaw = RobotContainer.driveSubsystem.getYaw();
      //double yawrate = RobotContainer.driveSubsystem.getTurnRate();

      // Update LLs with current YAW, so they can return correct position for Megatag2
      //LimelightHelpers.SetRobotOrientation(cn, yaw, yawrate, 0, 0, 0, 0);

      // Select the best coordinates
      if (LimelightHelpers.getTV(cn)) {
        

        PoseEstimate pe;

        pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cn);
               

        SmartDashboard.putNumber("LL-FIDN-"+cn,pe.rawFiducials.length);
        // Skip unreliable measuremements
        if (pe.rawFiducials.length>0) {
          SmartDashboard.putNumber("LL-ambig-"+cn,pe.rawFiducials[0].ambiguity);
        }
        
        
        if (! (pe.rawFiducials.length>0
        // && pe.rawFiducials[0].ambiguity>0.7
           )
         ) {
          continue;
        }

        // Now that we know the measurement is reliable, remember that we're actually seeing AT
        tvisible = true;
        if (ll4) { tvisibleLL4 = true; }


        if (pe.timestampSeconds > bestPoseTimestamp) { // update timestamps and poses for all-camera if needed
          bestPoseTimestamp = pe.timestampSeconds;
          bestPose = pe.pose;



        }

        if (ll4 && (pe.timestampSeconds > bestPoseTimestampLL4)) { // update timestamps and poses for LL4 if needed
              bestPoseTimestampLL4 = pe.timestampSeconds;
              bestPoseLL4 = pe.pose;

        }
      }
    } // end of loop checking whether the AT is visible

    bestVisible = tvisible;
    bestVisibleLL4 = tvisibleLL4;
    
    if(bestVisibleLL4) {


      // Set QuestNav initial pose
      // if (! RobotContainer.questNavSubsystem.isInitialPoseSet() && bestAmbiguity < maxBestAmbiguity) {
      //   RobotContainer.questNavSubsystem.resetQuestOdometry(bestPoseLL4);
      //   RobotContainer.questNavSubsystem.setInitialPoseSet(true); // Tell Quest subsystem that the initial pose of Quest is set
      //   RobotContainer.driveSubsystem.addVisionMeasurement(bestPoseLL4, Utils.fpgaToCurrentTime(bestPoseTimestampLL4)); // update robot odometry as well
      // }

      // Only reset Quest and odometry after initial setup during the game if some button is pressed
      //if (bestAmbiguity < maxBestAmbiguity) {
      //  RobotContainer.driveSubsystem.addVisionMeasurement(bestPoseLL4, Utils.fpgaToCurrentTime(bestPoseTimestampLL4));
      //}

    }
  }
}


