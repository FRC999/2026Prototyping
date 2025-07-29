// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIContants.ControllerDevice;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
  // public static final ArmSubsystem armSubsystem = new ArmSubsystem();

  public static Controller xboxDriveController = new Controller(ControllerDevice.XBOX_CONTROLLER);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    configureDriverInterface();

    driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis()));
  }

  private void configureDriverInterface() {}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // testArm();

    new JoystickButton(xboxDriveController, 2)
      .onTrue(new InstantCommand(() -> questNavSubsystem.resetQuestOdometry(new Pose2d(1, 2, Rotation2d.kZero)))
      .andThen(new InstantCommand(() -> driveSubsystem.setOdometryPoseToSpecificPose(new Pose2d(1, 2, Rotation2d.kZero))))
      );
  }

  // private void testArm() {
  //   new JoystickButton(xboxDriveController, 1)
  //     .onTrue(new InstantCommand(() -> armSubsystem.runArmMotors(0.1)))
  //     .onFalse(new InstantCommand(() -> armSubsystem.stopMotors()));

  //   new JoystickButton(xboxDriveController, 2)
  //     .onTrue(new InstantCommand(() -> armSubsystem.runArmMotors(0.1)))
  //     .onFalse(new InstantCommand(() -> armSubsystem.stopMotors()));
  // }

    // Driver preferred controls
    private double getDriverXAxis() {
      //return -xboxController.getLeftStickY();
      return xboxDriveController.getRightStickY();
    }
  
    private double getDriverYAxis() {
      //return -xboxController.getLeftStickX();
      return xboxDriveController.getRightStickX();
    }
  
    private double getDriverOmegaAxis() {
      //return -xboxController.getLeftStickOmega();
      return -xboxDriveController.getLeftStickX() * 0.6;
    }




    public static Command runTrajectory2PosesSlow(Pose2d startPose, Pose2d endPose,
      boolean shouldResetOdometryToStartingPose) {
    try {
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          AutoConstants.testPathCconstraints,
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
