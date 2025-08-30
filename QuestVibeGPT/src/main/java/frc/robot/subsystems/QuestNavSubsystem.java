// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;

import java.util.ArrayList;
import java.util.Objects;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.QuestCharacterizationHelper;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav;
   Transform2d ROBOT_TO_QUEST = new Transform2d(-0.32, -0.29, Rotation2d.k180deg); //Original was -0.32, -0.29
  // Transform2d ROBOT_TO_QUEST = new Transform2d(0, 0, Rotation2d.kZero); //Original was -0.32, -0.29
  //  final Transform2d ROBOT_TO_QUEST_CHARACTERIZATION = new Transform2d(0, 0, Rotation2d.kZero); //Use for characterization
  //  final Transform2d ROBOT_TO_QUEST_NORMAL = new Transform2d(-0.32, -0.29, Rotation2d.k180deg); //Original was -0.32, -0.29

  public static final Pose2d characterizationQuestPose = new Pose2d();
  public static int characterizationCounter = 0;
  public static boolean isCharacterizationRunning = false;
  public static Pose2d startCharacterizationPose;
  public static Pose2d endCharacterizationPose;
  public static double savedQuestAngle;

  Pose2d robotPose = new Pose2d(0, 0, Rotation2d.kZero);
  final Pose2d nullPose = new Pose2d(-1000, -1000, Rotation2d.kZero);

  Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill( 
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

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
      nullPose;
    
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
        nullPose;
  }

  public void resetQuestOdometry(Pose2d rP) {

    // Transform by the offset to get the Quest pose
    Pose2d questPose = rP.transformBy(ROBOT_TO_QUEST);

    // Send the reset operation
    questNav.setPose(questPose);
    System.out.println("Quest Odometry Reset To: " + questPose.toString());
    System.out.println("QRP: " + rP.toString());
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
                Pose2d pose = this.getQuestPose();
                // System.out.println("Get X: " + pose.getX() + " Get Y: " + pose.getY());
                if(!pose.equals(nullPose)) {
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
            var c = QuestCharacterizationHelper.estimateCircleCenter(robotPoses);
            System.out.println("X: " + (c.getX() - ROBOT_TO_QUEST.getX()) +
                               " Y: " + (c.getY() - ROBOT_TO_QUEST.getY()) +
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
              savedQuestAngle = this.getQuestPose().getRotation().getDegrees();
              questNav.setPose(characterizationQuestPose);
              },
            RobotContainer.driveSubsystem
        ),

        // Main loop: collect samples
        Commands.run(
            () -> {
             if (characterizationCounter++ == 10) {
              startCharacterizationPose = this.getQuestPose();
              RobotContainer.driveSubsystem.drive(driveSpeed, 0, 0);
             }
            },
            this // this subsystem (QuestNavSubsystem)
        )
        // Cleanup: estimate circle center
        .finallyDo(interrupted -> {
          try {
            endCharacterizationPose = this.getQuestPose();
            System.out.println("S: " + startCharacterizationPose.toString() +
                               " E: " + endCharacterizationPose.toString() +
                               " Angle: " + Math.atan((endCharacterizationPose.getY()-startCharacterizationPose.getY()) /
                                                    (endCharacterizationPose.getX()-startCharacterizationPose.getX())));
          } catch (Exception e) {
            System.out.println(e);
          }
          // optional: stop drivetrain
          RobotContainer.driveSubsystem.drive(0, 0, 0);
          questNav.setPose(new Pose2d(0,0,Rotation2d.fromDegrees(savedQuestAngle)));
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
        SmartDashboard.putString("qTranformedPose: ", getQuestRobotPose().toString());
        SmartDashboard.putString("qTruePose: ", getQuestPose().toString());
        SmartDashboard.putNumber("TimeStamp: ", getQTimeStamp());
        SmartDashboard.putNumber("TimeStampA: ", getQAppTimeStamp());
        SmartDashboard.putNumber("TimeStampFPGS: ", Utils.fpgaToCurrentTime(getQTimeStamp()));
      }


      //update pose Frames
      poseFrames = questNav.getAllUnreadPoseFrames();
      // Display number of frames provided
      SmartDashboard.putNumber("qFrames", poseFrames.length);
      if(SwerveConstants.CTR_ODOMETRY_UPDATE_FROM_QUEST && !isCharacterizationRunning) {
        for (PoseFrame questFrame : poseFrames) {
          // Get the pose of the Quest
          Pose2d questPose = questFrame.questPose();
          // Get timestamp for when the data was sent
          double timestamp = questFrame.dataTimestamp();

          // Transform by the mount pose to get your robot pose
          Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

          // You can put some sort of filtering here if you would like!

          // Add the measurement to our estimator
          RobotContainer.driveSubsystem.addVisionMeasurement(robotPose, timestamp, QUESTNAV_STD_DEVS);
        }
      }
    }
  }
}
