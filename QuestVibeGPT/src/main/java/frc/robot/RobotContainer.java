// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.QuestNavTrajectoryTest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;


public class RobotContainer {
    public static QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
    
     // kSpeedAt12Volts desired top speed
     // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(5);
    

    public static final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.createDrivetrain();

    public RobotContainer() {
        configureBindings();

        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drivetrain.getDrive().withVelocityX(-joystick.getLeftY() * SwerveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * SwerveConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * SwerveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> drivetrain.getIdle()).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> drivetrain.getBrake()));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            drivetrain.getPoint().withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().onTrue(new QuestNavTrajectoryTest())
                    .onFalse(stopRobotCommand());
    }

    public Command stopRobotCommand() {
        System.out.println("***Stopping Robot");
        return drivetrain.applyRequest(() ->
                drivetrain.getDrive().withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            );
    }

    public static Command runTrajectoryPathPlannerWithForceResetOfStartingPose(String tr,
      boolean shouldResetOdometryToStartingPose, boolean flipTrajectory) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(tr);

      if (flipTrajectory) {
        path = path.flipPath();
      }

      Pose2d startPose = path.getStartingHolonomicPose().get(); // reset odometry, as PP may not do so

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      if (! shouldResetOdometryToStartingPose) {
        return AutoBuilder.followPath(path);
      } else { // reset odometry the right way
        // return Commands.sequence(AutoBuilder.resetOdom(startPose), AutoBuilder.followPath(path));
        return Commands.sequence(AutoBuilder.resetOdom(startPose));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }



    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
