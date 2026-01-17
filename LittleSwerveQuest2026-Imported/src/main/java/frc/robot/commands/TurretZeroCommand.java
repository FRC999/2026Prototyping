package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;

/** Zeros turret to "forward" (0 degrees). */
public class TurretZeroCommand extends TurretGoToAngleCommand {
  public TurretZeroCommand(TurretSubsystem turret, double toleranceDeg) {
    super(turret, 0.0, toleranceDeg);
  }
}
