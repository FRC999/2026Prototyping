// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.EmptyHopperCommand;
import frc.robot.commands.EmptyShooterCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeToHopperCommand;
import frc.robot.commands.IntakeToShooterCommand;
import frc.robot.commands.HopperToShooterCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.StopIntakeProcessSequentialCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();
  public static HopperSubsystem hopperSubsystem = new HopperSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static CommandXboxController driverController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(
      
      new DriveManuallyCommand()
    );
    // Configure the trigger bindings
    configureBindings();
  }

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

    driverController.leftTrigger()
      .onTrue(new IntakeToHopperCommand())
      .onFalse(new StopIntakeProcessSequentialCommand());

    driverController.rightTrigger()
      .onTrue(new HopperToShooterCommand())
      .onFalse(new StopIntakeProcessSequentialCommand());

    driverController.rightBumper() 
      .onTrue(new IntakeToShooterCommand())
      .onFalse(new StopIntakeProcessSequentialCommand());

    driverController.b()
      .onTrue(new EmptyHopperCommand())
      .onFalse(new StopIntakeProcessSequentialCommand());
      
    driverController.a()
      .onTrue(new EmptyShooterCommand())
      .onFalse(new StopIntakeProcessSequentialCommand());
  }

  public static double getX(){
    return driverController.getLeftX();
  }

  public static double getY(){
    return driverController.getLeftY();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
