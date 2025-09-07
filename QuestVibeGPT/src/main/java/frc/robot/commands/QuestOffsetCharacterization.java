// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.lib.QuestCharacterizationHelper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class QuestOffsetCharacterization extends Command {

  double rotationalSpeed = (SwerveConstants.MaxAngularRate)/6.0;
  ArrayList<Double[]> robotPoses = new ArrayList<>();
  int everyN = 10;
  int counter = 0;

  /** Creates a new QuestOffsetCharacterization. */
  public QuestOffsetCharacterization() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPoses.clear();
    counter = 0;
    RobotContainer.driveSubsystem.drive(0, 0, rotationalSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(counter++ % everyN == 0) {
      Pose2d pose = RobotContainer.questNavSubsystem.getQuestPose();
      robotPoses.add(new Double[]{pose.getX(), pose.getY()});
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      var c = QuestCharacterizationHelper.estimateCircleCenter(robotPoses);
      System.out.println("X: " + c.getX() + " Y: " + c.getY() + " Num: " + robotPoses.size());
    }
    catch (Exception e) {
      System.out.println(e);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
