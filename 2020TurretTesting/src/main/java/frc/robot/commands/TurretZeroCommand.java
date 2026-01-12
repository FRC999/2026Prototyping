package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;

public class TurretZeroCommand extends InstantCommand {
  public TurretZeroCommand(TurretSubsystem turret) {
    super(turret::seedRelativeFromAbsolute, turret);
  }
}
