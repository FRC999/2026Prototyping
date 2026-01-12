// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SysId;
import frc.robot.commands.ShooterAdjustRpmCommand;
import frc.robot.commands.ShooterRunPIDCommand;
import frc.robot.commands.ShooterStopCommand;
import frc.robot.commands.TurretGoToAngleCommand;
import frc.robot.commands.TurretJogCommand;
import frc.robot.commands.TurretZeroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick joystick = new Joystick(Constants.OI.DRIVER_JOYSTICK_PORT);

  // Subsystems
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // You said you already have this subsystem; keep your real one.
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    // Shooter: start runs velocity PID, stop on release for safety
    new JoystickButton(joystick, Constants.OI.BTN_SHOOTER_START)
        .onTrue(new ShooterRunPIDCommand(shooterSubsystem))
        .onFalse(new ShooterStopCommand(shooterSubsystem));

    // Shooter explicit stop
    new JoystickButton(joystick, Constants.OI.BTN_SHOOTER_STOP)
        .onTrue(new ShooterStopCommand(shooterSubsystem));

    // Artillery table tuning: bump RPM up/down by 10
    new JoystickButton(joystick, Constants.OI.BTN_RPM_UP)
        .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem, +Constants.Shooter.RPM_STEP));

    new JoystickButton(joystick, Constants.OI.BTN_RPM_DOWN)
        .onTrue(new ShooterAdjustRpmCommand(shooterSubsystem, -Constants.Shooter.RPM_STEP));

    // Turret jog: hold button to move, release stops
    new JoystickButton(joystick, Constants.OI.BTN_TURRET_JOG_LEFT)
        .onTrue(new TurretJogCommand(turretSubsystem, +0.2))
        .onFalse(new TurretJogCommand(turretSubsystem, 0.0)); // stop

    new JoystickButton(joystick, Constants.OI.BTN_TURRET_JOG_RIGHT)
        .onTrue(new TurretJogCommand(turretSubsystem, -0.2))
        .onFalse(new TurretJogCommand(turretSubsystem, 0.0)); // stop

    // Turret zero (seed relative = 0 at abs=2900)
    new JoystickButton(joystick, Constants.OI.BTN_TURRET_ZERO)
        .onTrue(new TurretZeroCommand(turretSubsystem));

    // Optional: quick goto angles (hold-to-run, release stops)
    new JoystickButton(joystick, Constants.OI.BTN_TURRET_GOTO_0_DEG)
        .onTrue(new TurretGoToAngleCommand(turretSubsystem, 0.0, true))
        .onFalse(new TurretJogCommand(turretSubsystem, 0.0));

    new JoystickButton(joystick, Constants.OI.BTN_TURRET_GOTO_45_DEG)
        .onTrue(new TurretGoToAngleCommand(turretSubsystem, 45.0, true))
        .onFalse(new TurretJogCommand(turretSubsystem, 0.0));

    
    if (SysId.ENABLE_SYSID) {
      // ---------------- SysId bindings ----------------
      // WPILib recommends "hold to run" so you can abort instantly if something looks wrong.
      new JoystickButton(joystick, Constants.OI.BTN_SYSID_SHOOTER_QUASI_FWD)
          .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
          .onFalse(new ShooterStopCommand(shooterSubsystem));

      new JoystickButton(joystick, Constants.OI.BTN_SYSID_SHOOTER_QUASI_REV)
          .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
          .onFalse(new ShooterStopCommand(shooterSubsystem));

      new JoystickButton(joystick, Constants.OI.BTN_SYSID_SHOOTER_DYN_FWD)
          .whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward))
          .onFalse(new ShooterStopCommand(shooterSubsystem));

      // Expose the full set (including dynamic reverse + turret tests) on SmartDashboard as well.
      SmartDashboard.putData("SysId/Shooter Dynamic Reverse",
          shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      SmartDashboard.putData("SysId/Turret Quasistatic Forward",
          turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("SysId/Turret Quasistatic Reverse",
          turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      SmartDashboard.putData("SysId/Turret Dynamic Forward",
          turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("SysId/Turret Dynamic Reverse",
          turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      }

// Optional example: track a field point (tx,ty) while held
    // new JoystickButton(joystick, 10)
    //     .onTrue(new TrackFieldPointCommand(turretSubsystem, driveSubsystem, 8.0, 4.0, true))
    //     .onFalse(new TurretJogCommand(turretSubsystem, 0.0));
  }

  // Expose these if your Robot.java / autos need them
  public TurretSubsystem getTurretSubsystem() { return turretSubsystem; }
  public ShooterSubsystem getShooterSubsystem() { return shooterSubsystem; }
  public DriveSubsystem getDriveSubsystem() { return driveSubsystem; }

}
