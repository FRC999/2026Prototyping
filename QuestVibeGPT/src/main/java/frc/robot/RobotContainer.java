// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.Constants.OperatorConstants.OIContants.ControllerDevice;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.OneMeterForwardPPCommand;
import frc.robot.commands.OneMeterForwardTurnPPCommand;
import frc.robot.commands.QuestNavTrajectoryTest;
import frc.robot.commands.ThreeMeterForwardPPCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;


public class RobotContainer {
    public static QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();

     // kSpeedAt12Volts desired top speed
     // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

    private final Controller xboxDriveController = new Controller(ControllerDevice.XBOX_CONTROLLER);
    public static boolean isAllianceRed = false;
    public static boolean isReversingControllerAndIMUForRed = true;

    public static final DriveSubsystem driveSubsystem = DriveSubsystem.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        testTrajectory();

        driveSubsystem.setDefaultCommand(
      new DriveManuallyCommand(
          () -> getDriverXAxis(),
          () -> getDriverYAxis(),
          () -> getDriverOmegaAxis()));
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // driveSubsystem.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     driveSubsystem.applyRequest(() ->
        //         driveSubsystem.getDrive().withVelocityX(-xboxDriveController.getLeftY() * SwerveConstants.MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-xboxDriveController.getLeftX() * SwerveConstants.MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-xboxDriveController.getRightX() * SwerveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(driveSubsystem.applyRequest(() -> driveSubsystem.getIdle()).ignoringDisable(true));

        // xboxDriveController.a().whileTrue(driveSubsystem.applyRequest(() -> driveSubsystem.getBrake()));
        // xboxDriveController.b().whileTrue(driveSubsystem.applyRequest(() ->
        //     driveSubsystem.getPoint().withModuleDirection(new Rotation2d(-xboxDriveController.getLeftY(), -xboxDriveController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // xboxDriveController.back().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kForward));
        // xboxDriveController.back().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kReverse));
        // xboxDriveController.start().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kForward));
        // xboxDriveController.start().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // xboxDriveController.leftBumper().onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric()));

        driveSubsystem.registerTelemetry(logger::telemeterize);

        // xboxDriveController.x().onTrue(new QuestNavTrajectoryTest())
        //             .onFalse(stopRobotCommand());
    }

    public Command stopRobotCommand() {
        System.out.println("***Stopping Robot");
        return driveSubsystem.applyRequest(() ->
                driveSubsystem.getDrive().withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            );
    }

    private void testTrajectory() {
      new JoystickButton(xboxDriveController, 1)
        .onTrue(new OneMeterForwardTurnPPCommand());
        new JoystickButton(xboxDriveController, 2)
        .onTrue(new ThreeMeterForwardPPCommand());
    }

     // Driver preferred controls
     private double getDriverXAxis() {
      //return -xboxController.getLeftStickY();
      // SmartDashboard.putNumber("X-Axis: ", -xboxDriveController.getRightStickY());
      return -xboxDriveController.getRightStickY();
    }
  
    private double getDriverYAxis() {
      //return -xboxController.getLeftStickX();
      // SmartDashboard.putNumber("Y-Axis: ", -xboxDriveController.getRightStickX());
      return -xboxDriveController.getRightStickX();
    }
  
    private double getDriverOmegaAxis() {
      //return -xboxController.getLeftStickOmega();
      // SmartDashboard.putNumber("Z-Axis: ", -xboxDriveController.getLeftStickX() * 0.6);
      return -xboxDriveController.getLeftStickX() * 0.6;
    }

    public static Command runTrajectoryPathPlannerWithForceResetOfStartingPose(String tr,
      boolean shouldResetOdometryToStartingPose, boolean flipTrajectory) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(tr);

      // if (flipTrajectory) {
      //   path = path.flipPath();
      // }

      Pose2d startPose = path.getStartingHolonomicPose().get(); // reset odometry, as PP may not do so

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      if (! shouldResetOdometryToStartingPose) {
        return AutoBuilder.followPath(path);
      } else { // reset odometry the right way
        return Commands.sequence(AutoBuilder.resetOdom(startPose), AutoBuilder.followPath(path));
        //return Commands.sequence(AutoBuilder.resetOdom(startPose));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  // public static void setIfAllianceRed() {
  //   var alliance = DriverStation.getAlliance();
  //   if (! alliance.isPresent()) {
  //       System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
  //   } else {
  //       isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
  //       System.out.println("*** RED Alliance: "+isAllianceRed);
  //   }
  // }

  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }



    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
