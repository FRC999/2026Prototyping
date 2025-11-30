// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OdometryUpdates;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;

import frc.robot.OdometryUpdates.QuestNavConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import org.opencv.core.Point;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.QuestHelpers;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav;
  
  // Transform2d ROBOT_TO_QUEST = new Transform2d(0, 0, Rotation2d.kZero); //Original was -0.32, -0.29
  //  final Transform2d ROBOT_TO_QUEST_CHARACTERIZATION = new Transform2d(0, 0, Rotation2d.kZero); //Use for characterization
  //  final Transform2d ROBOT_TO_QUEST_NORMAL = new Transform2d(-0.32, -0.29, Rotation2d.k180deg); //Original was -0.32, -0.29

  public static int characterizationCounter = 0;
  public static boolean isCharacterizationRunning = false;
  public static Pose2d startCharacterizationPose;
  public static Pose2d endCharacterizationPose;
  public static double savedQuestAngle;

  
  private boolean initialPoseSet;
  public boolean isInitialPoseSet() {
    return initialPoseSet;
  }
  public void setInitialPoseSet(boolean initialPoseSet) {
    this.initialPoseSet = initialPoseSet;
  }
 

  PoseFrame[] poseFrames;

  /** Creates a new QuestNavSubsystem. */
  public QuestNavSubsystem() {
    initialPoseSet = false;
    questNav = new QuestNav();

    resetToZeroPose();

  }

  public void resetToZeroPose() {
    Pose3d questPose = (QuestNavConstants.robotZeroPose3d.transformBy(QuestNavConstants.ROBOT_TO_QUEST_3D));
    questNav.setPose(questPose);
    System.out.println("****QRobot reset to zero pose: " + questPose.toString());
  }



  /**
   * reset Quest YAW based on the specified Robot YAW
   * @param angle (degrees)
   */
  public void resetQuestIMUToAngle(double angle) {
    System.out.println("Quest Robot Pose: " + 
        java.util.Objects.requireNonNullElse(getQuestRobotPose2d(),"").toString());
    System.out.println("QAngle: " + angle);
    System.out.println("New QAngle: " + (Rotation2d.fromDegrees(angle).
        minus(getQuestRobotPose2d().getRotation())).getDegrees());
    
    Pose2d newRobotPose = new Pose2d(getQuestRobotPose2d().getTranslation(), Rotation2d.fromDegrees(angle));
    System.out.println(newRobotPose.toString());
    questNav.setPose(new Pose3d(newRobotPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST)));
  }

  /**
   * Gets the yaw of the QuestNav (Z axis rotation) (yaw is the direction that the
   * questnav is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getTrueYaw() {
    return getQuestPose2d().getRotation().getDegrees(); // With True QuestNav Pose this method returns values in degrees
  }

  /**
   * Get robot chassis YAW based on the latest Quest-transformed robot pose
   * @return YAW (degrees)
   */
  public double getQuestRobotYaw() {
    return getQuestRobotPose2d().getRotation().getDegrees();
  }



  /**
   * Zeroes the yaw of the robot
   * assuming the robot is pointing AWAY from the drivers
   * (meaning - angke is set 180 for RED)
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double previousYaw = getQuestRobotYaw();
    if (RobotContainer.isAllianceRed
        && RobotContainer.isReversingControllerAndIMUForRed) {
      resetQuestIMUToAngle(180);
    } else {
      resetQuestIMUToAngle(0);
    }
    System.out.println("New Yaw: " + getQuestRobotYaw());
    return previousYaw;
  }

  /**
   * get the latest Robot pose transforned from Quest pose
   * from the uncreared frames in Quest buffer
   * @return - Quest Pose2D
   */
  public Pose2d getQuestRobotPose2d() {
    return (poseFrames != null && poseFrames.length > 0) ?
      poseFrames[poseFrames.length - 1].questPose3d().toPose2d()
        .transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse()) :
        QuestNavConstants.nullPose;
  }

    /**
   * get the latest Robot pose transforned from Quest pose
   * from the uncreared frames in Quest buffer
   * @return - Quest Pose3D
   */
  public Pose3d getQuestRobotPose3d() {
    return (poseFrames != null && poseFrames.length > 0) ?
      poseFrames[poseFrames.length - 1].questPose3d()
        .transformBy(QuestNavConstants.ROBOT_TO_QUEST_3D.inverse()) :
        QuestNavConstants.nullPose3d;
  }

  /**
   * get the data timestamp of the latest Quest pose from the uncleared frames in Quest buffer
   * @return - raw timestamp
   */
  public double getQTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ?
      poseFrames[poseFrames.length - 1].dataTimestamp() :
      0;
  }

  /**
   * get the app timestamp of the latest Quest pose from the uncleared frames in Quest buffer
   * @return - raw timestamp
   */
  public double getQAppTimeStamp() {
    return (poseFrames != null && poseFrames.length > 0) ?
        poseFrames[poseFrames.length - 1].appTimestamp() :
        0;
  }

  /**
   * get the latest Quest pose from the uncreared frames in Quest buffer
   * @return - Quest Pose2D
   */
  public Pose2d getQuestPose2d() {
    return (poseFrames != null && poseFrames.length > 0) ?
        poseFrames[poseFrames.length - 1].questPose3d().toPose2d() :
        QuestNavConstants.nullPose;
  }

    /**
   * get the latest Quest pose from the uncreared frames in Quest buffer
   * @return - Quest Pose2D
   */
  public Pose3d getQuestPose3d() {
    return (poseFrames != null && poseFrames.length > 0) ?
        poseFrames[poseFrames.length - 1].questPose3d() :
        QuestNavConstants.nullPose3d;
  }

  /**
   * Reset Quest pose based on specified Robot pose
   * @param rP - robot pose
   */
  public void resetQuestOdometry(Pose3d rP) {

    // Transform by the offset to get the Quest pose
    Pose3d questPose = rP.transformBy(QuestNavConstants.ROBOT_TO_QUEST_3D);

    // Send the reset operation
    questNav.setPose(questPose);
    if(DebugTelemetrySubsystems.questnav){
      System.out.println("*** Quest Odometry Reset To: " + questPose.toString());
      System.out.println("*** QRP: " + rP.toString());
    }
  }

  /**
   * Creates a command that rotates the robot and characterizes the Quest translation offset.
   * 
   * @return Command that runs indefinitely until canceled.
   */
  public Command offsetTranslationCharacterizationCommand() {
    double rotationalSpeed = SwerveConstants.MaxAngularRate / 6.0;
    ArrayList<Double[]> robotPoses = new ArrayList<>();
    int everyN = 10;

    return Commands.sequence(
        // Initialize: start rotating
        Commands.runOnce(
            () -> {
              robotPoses.clear();
              characterizationCounter = 0;
              RobotContainer.driveSubsystem.drive(0, 0, rotationalSpeed);},
            RobotContainer.driveSubsystem
        ),

        // Main loop: collect samples
        Commands.run(
            () -> {
              if (characterizationCounter++ % everyN == 0) {
                Pose2d pose = this.getQuestPose2d();
                // System.out.println("Get X: " + pose.getX() + " Get Y: " + pose.getY());
                if(!pose.equals(QuestNavConstants.nullPose)) {
                  robotPoses.add(new Double[]{pose.getX(), pose.getY()});
                //  System.out.println("Valid");
                }
              }
            },
            this // this subsystem (QuestNavSubsystem)
        )
        // Cleanup: estimate circle center
        .finallyDo(interrupted -> {
          try {
            var c = QuestHelpers.estimateCircleCenter(robotPoses);
            System.out.println("X: " + (c.getX() - QuestNavConstants.ROBOT_TO_QUEST.getX()) +
                               " Y: " + (c.getY() - QuestNavConstants.ROBOT_TO_QUEST.getY()) +
                               " Num: " + robotPoses.size());
          } catch (Exception e) {
            System.out.println(e);
          }
          // optional: stop drivetrain
          RobotContainer.driveSubsystem.drive(0, 0, 0);
        })
    )
    // Never ends on its own
    .until(() -> false);
  }

  /**
   * Creates a command that drives the robot straight and characterizes the Quest angle offset.
   * 
   * @return Command that runs indefinitely until canceled.
   */
  public Command offsetAngleCharacterizationCommand() {
    double driveSpeed = SwerveConstants.MaxSpeed / 6.0;

    return Commands.sequence(
        // Initialize: start rotating
        Commands.runOnce(
            () -> {
              isCharacterizationRunning = true;
              characterizationCounter = 0;
              savedQuestAngle = this.getQuestPose2d().getRotation().getDegrees();
              questNav.setPose(new Pose3d(QuestNavConstants.characterizationQuestPose));
              },
            RobotContainer.driveSubsystem
        ),

        // Main loop: collect samples
        Commands.run(
            () -> {
             if (characterizationCounter++ == 10) {
              startCharacterizationPose = this.getQuestPose2d();
              RobotContainer.driveSubsystem.drive(driveSpeed, 0, 0);
             }
            },
            this // this subsystem (QuestNavSubsystem)
        )
        // Cleanup: estimate circle center
        .finallyDo(interrupted -> {
          try {
            endCharacterizationPose = this.getQuestPose2d();
            System.out.println("S: " + startCharacterizationPose.toString() +
                               " E: " + endCharacterizationPose.toString() +
                               " Angle: " + Math.atan((endCharacterizationPose.getY()-startCharacterizationPose.getY()) /
                                                    (endCharacterizationPose.getX()-startCharacterizationPose.getX())));
          } catch (Exception e) {
            System.out.println(e);
          }
          // optional: stop drivetrain
          RobotContainer.driveSubsystem.drive(0, 0, 0);
          questNav.setPose(new Pose3d((new Pose2d(0,0,Rotation2d.fromDegrees(savedQuestAngle)))));
          isCharacterizationRunning = false;
        })
    )
    // Never ends on its own
    .until(() -> false);
  }

  /**
   * get all Quest pose frames, clear unread queue
   * @return PoseFrame[]
   */
  public PoseFrame[] getAllUnreadPoseFrames() {
    // Delegate straight to the SDK. This call should return and CLEAR the unread queue.
    return questNav.getAllUnreadPoseFrames();
  }

  /** Return a copy of all current PoseFrame elements */
  public PoseFrame[] getAllCurrentPoseframes() {
    if (poseFrames == null) return null;
      return Arrays.stream(poseFrames)
        .map(pf -> pf == null ? null :
          new PoseFrame(
            new Pose3d(pf.questPose3d().getTranslation(), pf.questPose3d().getRotation()),
            pf.dataTimestamp(),
            pf.appTimestamp(),
            pf.frameCount()
          )
        )
      .toArray(PoseFrame[]::new);
  }

  /**
   * Check if Quest track poses
   * @return boolean
   */
  public boolean isTracking() {
    // Optional: surface vendor health so you can light up a dashboard indicator.
    try {
      return questNav.isTracking();
    } catch (Throwable t) {
      return false;
    }
  }

  @Override
  public void periodic() {

    if (questNav.isTracking()) {
      // This method will be called once per scheduler run

      // QuestNav telemetry
      if (DebugTelemetrySubsystems.questnav) {
        SmartDashboard.putString("qTranformedPose3dTranslation: ", getQuestRobotPose3d().getTranslation().toString());
        SmartDashboard.putNumber("qTranformedPose3dRotation: ", (getQuestRobotPose3d().getRotation().getMeasureZ().magnitude()*180/Math.PI));
        SmartDashboard.putString("qTruePose3dTranslation: ", getQuestPose3d().getTranslation().toString());
        SmartDashboard.putNumber("qTruePose3dRotation: ", (getQuestPose3d().getRotation().getMeasureZ().magnitude()*180/Math.PI));
        SmartDashboard.putNumber("TimeStamp: ", getQTimeStamp());
        SmartDashboard.putNumber("TimeStampA: ", getQAppTimeStamp());
        SmartDashboard.putNumber("TimeStampFPGS: ", Utils.fpgaToCurrentTime(getQTimeStamp()));
        SmartDashboard.putNumber("Time FPGA: ", Timer.getFPGATimestamp());
        if(poseFrames != null) {
          SmartDashboard.putNumber("qFrames", poseFrames.length);
        }
      }


      //update pose Frames
      poseFrames = questNav.getAllUnreadPoseFrames();

 
      // // Display number of frames provided
      // if(SwerveConstants.CTR_ODOMETRY_UPDATE_FROM_QUEST && !isCharacterizationRunning) {
      //   for (PoseFrame questFrame : poseFrames) {
      //     // Get the pose of the Quest
      //     Pose2d questPose = questFrame.questPose();
      //     // Get timestamp for when the data was sent
      //     double timestamp = questFrame.dataTimestamp();

      //     // Transform by the mount pose to get your robot pose
      //     Pose2d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

      //     // You can put some sort of filtering here if you would like!

      //     // Add the measurement to our estimator
      //     RobotContainer.driveSubsystem.addVisionMeasurement(robotPose, timestamp, QuestNavConstants.QUESTNAV_STD_DEVS);
      //   }
      // }
    }
  }
}
