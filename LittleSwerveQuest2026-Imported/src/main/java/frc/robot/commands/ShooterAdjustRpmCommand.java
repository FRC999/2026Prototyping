package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

/** Adjusts the shooter RPM setpoint by a delta (RPM). */
public class ShooterAdjustRpmCommand extends InstantCommand {
  public ShooterAdjustRpmCommand(ShooterSubsystem shooter, double deltaRpm) {
    super(() -> shooter.setTargetRpm(shooter.getTargetRpm() + deltaRpm), shooter);
  }
}
