package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/** Applies a constant duty-cycle to the turret while scheduled. */
public class TurretJogCommand extends Command {
  private final TurretSubsystem turret;
  private final double duty;

  public TurretJogCommand(TurretSubsystem turret, double duty) {
    this.turret = turret;
    this.duty = duty;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.setDutyCycle(duty);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }
}
