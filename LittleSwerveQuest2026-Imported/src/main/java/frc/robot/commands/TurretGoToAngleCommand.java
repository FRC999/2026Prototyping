package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/** Commands the turret to go to a target angle (deg) and ends when reached. */
public class TurretGoToAngleCommand extends Command {
  private final TurretSubsystem turret;
  private final double targetDeg;
  private final double toleranceDeg;

  public TurretGoToAngleCommand(TurretSubsystem turret, double targetDeg, double toleranceDeg) {
    this.turret = turret;
    this.targetDeg = targetDeg;
    this.toleranceDeg = toleranceDeg;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.goToAngleDeg(targetDeg);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Turret to Angle " + targetDeg + " " + interrupted + " with tolerance " + toleranceDeg);
  }

  @Override
  public boolean isFinished() {
    return turret.atAngleDeg(targetDeg, toleranceDeg);
  }
}
