// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.OdometryUpdates.LLAprilTagConstants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.lib.VisionHelpers;
import frc.robot.OdometryUpdates.LLAprilTagConstants.VisionHelperConstants.RobotPoseConstants;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopPanToAprilTag extends SequentialCommandGroup {
  /** Creates a new TeleopPanToAprilTag. */
  public TeleopPanToAprilTag(boolean resetOdometry) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand(
        () -> {
          try {
          return new PrintCommand("To April Tag")
            .andThen(
              RobotContainer.runTrajectory2PosesSlow(
                RobotContainer.driveSubsystem.getPose(),
                RobotPoseConstants.visionRobotPoses.get(
                 RobotPoseConstants.tagNumberToKey.get(
                   RobotPoseConstants.reefTagPoses.get(
                     VisionHelpers.getClosestReefTagToRobot(RobotContainer.llAprilTagSubsystem.getBestPoseFromAllLL())
                ))),
                resetOdometry));
          }
          catch (Exception e) {
            return new PrintCommand("BAD");
          }
          }
      ,Set.of())
    );
  }
}
