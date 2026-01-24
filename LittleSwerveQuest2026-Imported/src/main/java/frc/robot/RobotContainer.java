// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants.OIContants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.OdometryUpdates.LLAprilTagSubsystem;
import frc.robot.OdometryUpdates.OdometryUpdatesSubsystem;
import frc.robot.OdometryUpdates.QuestNavSubsystem;
import frc.robot.commands.BLUE_ElasticMultiplePaths;
import frc.robot.commands.BLUE_ElasticTestPathCommand;
import frc.robot.commands.BLUE_OneMeterForwardPPCommand;
import frc.robot.commands.BLUE_ThreeMeterForwardPPCommand;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.RED_OneMeterForwardTurnPPCommand;
import frc.robot.commands.ReturnTestPPCommand;
import frc.robot.commands.ShooterAdjustRpmCommand;
import frc.robot.commands.ShooterEnableCommand;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TurretJogCommand;
import frc.robot.commands.TurretZeroCommand;
import frc.robot.lib.ElasticHelpers;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.FuelSim;
import frc.robot.util.TurretSimVisualizer;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.ShootFuelSimCommand;



public class RobotContainer {

  // kSpeedAt12Volts desired top speed
  // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // Use open-loop control for drive motors
  private TurretSimVisualizer turretSimVisualizer;

  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private final Joystick turretStick =  new Joystick(0);
  private final Controller xboxDriveController = new Controller(OIContants.XBOX_CONTROLLER);
  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;

  public static final DriveSubsystem driveSubsystem = DriveSubsystem.createDrivetrain();
  public static QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
  public static LLAprilTagSubsystem llAprilTagSubsystem = new LLAprilTagSubsystem();
  public static OdometryUpdatesSubsystem odometryUpdateSubsystem = new OdometryUpdatesSubsystem();
  public static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();

  // Turret + shooter subsystems
  public static final TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static final Pose2d randomStartPose = new Pose2d(1.510, 4.978, new Rotation2d());
  public static final Pose2d randomEndPose = new Pose2d(4.00, 6.00, new Rotation2d());
  private static String lastAutoPathPreview = "";

  

  public RobotContainer() {
    // Runtime gate for SysId routines. Must be enabled intentionally.
    SmartDashboard.setDefaultBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);

    new EventTrigger("TrajectoryOnTheFly").onTrue(runTrajectory2Poses(randomStartPose, randomEndPose, true));
    configureBindings();
    testTrajectory();
    setYaws();

    driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis()));
    FollowPathCommand.warmupCommand().schedule();

    AutonomousConfigure();

    // if (RobotBase.isSimulation()) {
    //   new JoystickButton().onTrue(
    //     new ShootFuelSimCommand(driveSubsystem, turretSubsystem, shooterSubsystem)
    // );
    //}


  }

  public TurretSimVisualizer getTurretSimVisualizer() {
    return turretSimVisualizer;
  }

  private void configureSimulation() {
  // Start FuelSim (Hammerheads style)
  FuelSim sim = FuelSim.getInstance();
  sim.spawnStartingFuel(); // optional; delete if you don’t want balls pre-spawned

  sim.registerRobot(
      Constants.SimConstants.ROBOT_WIDTH_M,
      Constants.SimConstants.ROBOT_LENGTH_M,
      Constants.SimConstants.BUMPER_HEIGHT_M,
      driveSubsystem::getPose,
      driveSubsystem::getFieldSpeeds);

  sim.start();

  // Turret visualization in AdvantageScope
  turretSimVisualizer = new TurretSimVisualizer(driveSubsystem, turretSubsystem);
}

  public static void AutonomousConfigure() {
    SignalLogger.start();
    // port autonomous routines as commands
    // sets the default option of the SendableChooser to the simplest autonomous
    // command. (from touching the hub, drive until outside the tarmac zone)
    selectAutoListBasedOnATPose(driveSubsystem.getState().Pose);
    SmartDashboard.putData(autoChooser);
    autoChooser.addOption("TrajectoryOnTheFly", runTrajectory2Poses(randomStartPose, randomEndPose, true));
  }

  public static void selectAutoListBasedOnATPose(Pose2d robotPose2d) {
    boolean isRobotBlue = true;
    if (robotPose2d.getX() < TrajectoryHelper.FIELD_LENGTH_BLUE) {
      isRobotBlue = true;
    } else {
      isRobotBlue = false;
    }
    try {
      if (isRobotBlue) {
        autoChooser.addOption("One Meter Forward", new BLUE_OneMeterForwardPPCommand());
        autoChooser.addOption("ThreeMeterForward", new BLUE_ThreeMeterForwardPPCommand());
        autoChooser.addOption("Elastic Test Command", new BLUE_ElasticTestPathCommand());
        autoChooser.addOption("Elastic MultiplePaths Command", new BLUE_ElasticMultiplePaths());
        autoChooser.setDefaultOption("One Meter Forward", new BLUE_OneMeterForwardPPCommand());
      } else {
        autoChooser.addOption("1M->Turn", new RED_OneMeterForwardTurnPPCommand());
      }
    } catch (Exception e) {
      DriverStation.reportError("Error loading default auto path: " + e.getMessage(), e.getStackTrace());
    }
  }

  // NEW: call this periodically to keep the auto preview in sync with the chooser
  public static void updateAutoPathPreview() {
    Command selected = autoChooser.getSelected();
    if (selected == null) {
      return;
    }

    // Map the selected Command to its PathPlanner path file name
    ArrayList<String> newPathNameList = new ArrayList<String>();
    String newPathName = null;

    if (selected instanceof BLUE_OneMeterForwardPPCommand) {
      newPathName = "OneMeterForward";
      newPathNameList.add("OneMeterForward");
    } else if (selected instanceof RED_OneMeterForwardTurnPPCommand) {
      newPathName = "OneMeterForwardTurn";
      newPathNameList.add("OneMeterForwardTurn");
    } else if (selected instanceof BLUE_ThreeMeterForwardPPCommand) {
      newPathName = "ThreeMeterForward";
      newPathNameList.add("ThreeMeterForward");
    } else if (selected instanceof BLUE_ElasticTestPathCommand) {
      newPathName = "ElasticTestPath";
      newPathNameList.add("ElasticTestPath");
    } else if (selected instanceof BLUE_ElasticMultiplePaths) {
      newPathName = "ElasticMultiplePaths";
      newPathNameList.add("ElasticSegment1");
      newPathNameList.add("ElasticSegment2");
      newPathNameList.add("ElasticSegment3");
    }

    // Nothing we know how to preview
    if (newPathName == null) {
      return;
    }

    // If it's the same path as last time, don't spam reload
    if (newPathName.equals(lastAutoPathPreview)) {
      return;
    }

    lastAutoPathPreview = newPathName;

    try {
      List<PathPlannerPath> newPaths = new ArrayList<>();
      for (String path : newPathNameList) {
        newPaths.add(PathPlannerPath.fromPathFile(path));
      }
      ElasticHelpers.setAutoPathMultiple(newPaths); // pushes it into the auto Field2d
    } catch (Exception e) {
      DriverStation.reportError(
          "Error loading path for auto preview: " + newPathNameList,
          e.getStackTrace());
    }
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

    testTurretShooter();

    // xboxDriveController.x().onTrue(new QuestNavTrajectoryTest())
    // .onFalse(stopRobotCommand());
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
  
  public Command stopRobotCommand() {
    System.out.println("***Stopping Robot");
    return driveSubsystem.applyRequest(() -> driveSubsystem.getDrive().withVelocityX(0) // Drive forward with negative Y
                                                                                        // (forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)

    );
  }

  private void testTrajectory() {
    new JoystickButton(xboxDriveController, 1)
        .onTrue(new BLUE_OneMeterForwardPPCommand());
    new JoystickButton(xboxDriveController, 2)
    .onTrue(new BLUE_ThreeMeterForwardPPCommand());
    // new JoystickButton(xboxDriveController, 2)
    //     .onTrue(new ReturnTestPPCommand())
    //     .onFalse(new StopRobot());
    new JoystickButton(xboxDriveController, 3)
        .onTrue(new InstantCommand(
            () -> questNavSubsystem.resetQuestOdometry(new Pose3d(10, 10, 0, new Rotation3d(0, 0, Math.PI))))); // TODO:
                                                                                                                // Check
                                                                                                                // Formatting

    new JoystickButton(xboxDriveController, 4)
        .onTrue(questNavSubsystem.offsetTranslationCharacterizationCommand())
        .onFalse(new StopRobot());

    new JoystickButton(xboxDriveController, 5)
        .onTrue(questNavSubsystem.offsetAngleCharacterizationCommand())
        .onFalse(new StopRobot());
  }

  private void testTurretShooter() {
     // ---------------- Turret + Shooter prototype bindings (Joystick port 0) ----------------
    // Shooter: button 3 = run shooter (at current target, or DEFAULT_RPM). Release stops.
    new JoystickButton(turretStick, 3).whileTrue(new ShooterEnableCommand(shooterSubsystem));

    // Shooter: button 1/2 adjust RPM setpoint by +/- RPM_STEP.
    new JoystickButton(turretStick, 1)
        .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem, Constants.OperatorConstants.Shooter.RPM_STEP));
    new JoystickButton(turretStick, 2)
        .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem, -Constants.OperatorConstants.Shooter.RPM_STEP));

    // Turret: button 5/6 jog left/right with a gentle duty cycle; release stops.
    new JoystickButton(turretStick, 5).whileTrue(new TurretJogCommand(turretSubsystem, 0.25));
    new JoystickButton(turretStick, 6).whileTrue(new TurretJogCommand(turretSubsystem, -0.25));

    // Turret: button 4 = zero to forward (0 deg) and finish when reached.
    new JoystickButton(turretStick, 4).onTrue(new TurretZeroCommand(turretSubsystem, 0.5));

    // SysId (gated by Constants.SysId.ENABLE_SYSID):
    // Turret: button 7/8 quasistatic fwd/rev, 9/10 dynamic fwd/rev.
    new JoystickButton(turretStick, 7).whileTrue(turretSubsystem.sysIdQuasistatic(Direction.kForward));
    new JoystickButton(turretStick, 8).whileTrue(turretSubsystem.sysIdQuasistatic(Direction.kReverse));
    new JoystickButton(turretStick, 9).whileTrue(turretSubsystem.sysIdDynamic(Direction.kForward));
    new JoystickButton(turretStick, 10).whileTrue(turretSubsystem.sysIdDynamic(Direction.kReverse));

    // Shooter SysId: button 11/12 quasistatic fwd/rev (dynamic can be added if you reassign buttons).
    new JoystickButton(turretStick, 11).whileTrue(shooterSubsystem.sysIdQuasistatic(Direction.kForward));
    new JoystickButton(turretStick, 12).whileTrue(shooterSubsystem.sysIdQuasistatic(Direction.kReverse));
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
}
