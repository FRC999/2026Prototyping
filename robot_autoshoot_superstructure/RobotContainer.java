// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants.OIContants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.OdometryUpdates.LLAprilTagSubsystem;
import frc.robot.OdometryUpdates.OdometryUpdatesSubsystem;
import frc.robot.OdometryUpdates.QuestNavSubsystem;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ShooterAdjustRpmCommand;
import frc.robot.commands.AutoShootUntilEmpty;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TurretJogCommand;
import frc.robot.lib.ElasticHelpers;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

public class RobotContainer {

  // kSpeedAt12Volts desired top speed
  // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // Use open-loop control for drive motors
  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private final Controller xboxDriveController = new Controller(OIContants.XBOX_CONTROLLER);
  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;
  private static final Joystick turretStick =  new Joystick(0);

  public static final DriveSubsystem driveSubsystem = DriveSubsystem.createDrivetrain();
  public static QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
  public static LLAprilTagSubsystem llAprilTagSubsystem = new LLAprilTagSubsystem();
  public static OdometryUpdatesSubsystem odometryUpdateSubsystem = new OdometryUpdatesSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static HopperSubsystem hopperSubsystem = new HopperSubsystem();
  public static SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  public static TransferSubsystem transferSubsystem = new TransferSubsystem();
  public static AutoShootSupervisorSubsystem autoShootSupervisorSubsystem = new AutoShootSupervisorSubsystem(driveSubsystem, turretSubsystem, shooterSubsystem, spindexerSubsystem, transferSubsystem);
  public static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();

  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    setYaws();
    

    driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis()));
    FollowPathCommand.warmupCommand().schedule();

    AutonomousConfigure();
    testTurretShooter();

    // Default hopper estimate (used until you add beam breaks / indexer sensors)
    SmartDashboard.putNumber("Hopper/BallsEstimate", Constants.OperatorConstants.AutoShoot.DEFAULT_BALLS_ESTIMATE);
  }

  public static void AutonomousConfigure() {
    // port autonomous routines as commands
    // sets the default option of the SendableChooser to the simplest autonomous
    // command. (from touching the hub, drive until outside the tarmac zone)
    SmartDashboard.putData(autoChooser);
  }

  

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // driveSubsystem.setDefaultCommand(
    // // Drivetrain will execute this command periodically
    // driveSubsystem.applyRequest(() ->
    // driveSubsystem.getDrive().withVelocityX(-xboxDriveController.getLeftY() *
    // SwerveConstants.MaxSpeed) // Drive forward with negative Y (forward)
    // .withVelocityY(-xboxDriveController.getLeftX() * SwerveConstants.MaxSpeed) //
    // Drive left with negative X (left)
    // .withRotationalRate(-xboxDriveController.getRightX() *
    // SwerveConstants.MaxAngularRate) // Drive counterclockwise with negative X
    // (left)
    // )
    // );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled()
        .whileTrue(driveSubsystem.applyRequest(() -> driveSubsystem.getIdle()).ignoringDisable(true));

    // xboxDriveController.a().whileTrue(driveSubsystem.applyRequest(() ->
    // driveSubsystem.getBrake()));
    // xboxDriveController.b().whileTrue(driveSubsystem.applyRequest(() ->
    // driveSubsystem.getPoint().withModuleDirection(new
    // Rotation2d(-xboxDriveController.getLeftY(), -xboxDriveController.getLeftX()))
    // ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // xboxDriveController.back().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kForward));
    // xboxDriveController.back().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kReverse));
    // xboxDriveController.start().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kForward));
    // xboxDriveController.start().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kReverse));

    // // reset the field-centric heading on left bumper press
    // xboxDriveController.leftBumper().onTrue(driveSubsystem.runOnce(() ->
    // driveSubsystem.seedFieldCentric()));

    driveSubsystem.registerTelemetry(logger::telemeterize);

    // xboxDriveController.x().onTrue(new QuestNavTrajectoryTest())
    // .onFalse(stopRobotCommand());
  }

  public Command stopRobotCommand() {
    System.out.println("***Stopping Robot");
    return driveSubsystem.applyRequest(() -> driveSubsystem.getDrive().withVelocityX(0) // Drive forward with negative Y
                                                                                        // (forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)

    );
  }


  public void setYaws() {
    new JoystickButton(xboxDriveController, 8)
        .onTrue(new InstantCommand(() -> driveSubsystem.zeroChassisYaw())
            .andThen(new InstantCommand(() -> questNavSubsystem.zeroYaw())));
    new JoystickButton(xboxDriveController, 7)
        .onTrue(new InstantCommand(() -> questNavSubsystem.resetToZeroPose()));
  }

  // Driver preferred controls
  private double getDriverXAxis() {
    // return -xboxController.getLeftStickY();
    // SmartDashboard.putNumber("X-Axis: ", -xboxDriveController.getRightStickY());
    return -xboxDriveController.getRightStickY();
  }

  private double getDriverYAxis() {
    // return -xboxController.getLeftStickX();
    // SmartDashboard.putNumber("Y-Axis: ", -xboxDriveController.getRightStickX());
    return -xboxDriveController.getRightStickX();
  }

  private double getDriverOmegaAxis() {
    // return -xboxController.getLeftStickOmega();
    // SmartDashboard.putNumber("Z-Axis: ", -xboxDriveController.getLeftStickX() *
    // 0.6);
    return -xboxDriveController.getLeftStickX() * 0.6;
  }

  public static Command runTrajectoryPathPlannerWithForceResetOfStartingPose(String tr,
      boolean shouldResetOdometryToStartingPose, boolean flipTrajectory) {

    // alex test
    //System.out.println("Start drive routine");

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(tr);

      ElasticHelpers.setAutoPathSingle(path);

      Pose2d startPose = path.getStartingHolonomicPose().get(); // reset odometry, as PP may not do so

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      if (!shouldResetOdometryToStartingPose) {

        // alex test
        // System.out.println("Rigth before driving without reset");
        return AutoBuilder.followPath(path);

      } else { // reset odometry the right way

        // alex test
        // System.out.println("Rigth before driving with reset");

        return Commands.sequence(
            new InstantCommand(
                () -> questNavSubsystem.resetQuestOdometry(new Pose3d(TrajectoryHelper.flipQuestPoseRed(startPose)))),
            AutoBuilder.resetOdom(startPose), new WaitCommand(0), AutoBuilder.followPath(path));

        // return Commands.sequence(AutoBuilder.resetOdom(startPose));

        // return Commands.sequence(new InstantCommand(() ->
        // questNavSubsystem.resetQuestOdometry(TrajectoryHelper.flipQuestPoseRed(startPose))),
        // AutoBuilder.resetOdom(startPose));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

    public static Command runTrajectory2Poses(Pose2d startPose, Pose2d endPose,
      boolean shouldResetOdometryToStartingPose) {
    try {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          AutoConstants.pathCconstraints,
          new IdealStartingState(0, startPose.getRotation()),
          new GoalEndState(0, endPose.getRotation()));
      path.preventFlipping = true;
      driveSubsystem.setOdometryPoseToSpecificPose(startPose); // reset odometry, as PP may not do so
      if (!shouldResetOdometryToStartingPose) {
        return AutoBuilder.followPath(path);
      } else { // reset odometry the right way
        System.out.println("== Driving from "+startPose+" to "+endPose);
        return Commands.sequence(AutoBuilder.resetOdom(startPose), AutoBuilder.followPath(path));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  // alex test
  // public static Command testCommand2() {
  // return new PrintCommand("Test 2 Command");
  // }

  // Alliance color determination
  public void checkAllianceColor() {
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
  }

  public static void setIfAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (!alliance.isPresent()) {
      System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
      isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
      System.out.println("*** RED Alliance: " + isAllianceRed);
    }
  }

  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static void testTurretShooter() {
    // new JoystickButton(turretStick, 1)
    //     .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem, Constants.OperatorConstants.Shooter.RPM_STEP));

    // new JoystickButton(turretStick, 2)
    //     .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem, -Constants.OperatorConstants.Shooter.RPM_STEP));

    // Hold button 3: aim + spin up + feed until our ball estimate reaches 0.
    // NOTE: this uses ShooterSubsystem dip detection as a "ball fired" event.
    new JoystickButton(turretStick, 3).whileTrue(new AutoShootUntilEmpty(
        autoShootSupervisorSubsystem,
        turretSubsystem,
        shooterSubsystem,
        spindexerSubsystem,
        transferSubsystem));


    new JoystickButton(turretStick, 5).whileTrue(new TurretJogCommand(turretSubsystem, -0.25));
    new JoystickButton(turretStick, 6).whileTrue(new TurretJogCommand(turretSubsystem, 0.25));
  }
}
