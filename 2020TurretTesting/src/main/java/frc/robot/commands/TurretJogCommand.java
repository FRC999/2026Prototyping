package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretJogCommand extends Command {
  private final TurretSubsystem turret;
  private final double percent;

  public TurretJogCommand(TurretSubsystem turret, double percent) {
    this.turret = turret;
    this.percent = percent;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.setOpenLoopPercent(percent);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }
}
