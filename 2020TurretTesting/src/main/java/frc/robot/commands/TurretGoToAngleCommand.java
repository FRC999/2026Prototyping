package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretGoToAngleCommand extends Command {
  private final TurretSubsystem turret;
  private final double targetDeg;
  private final boolean useMotionMagic;

  private final Timer timer = new Timer();

  public TurretGoToAngleCommand(TurretSubsystem turret, double targetDeg, boolean useMotionMagic) {
    this.turret = turret;
    this.targetDeg = targetDeg;
    this.useMotionMagic = useMotionMagic;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    timer.restart();
    turret.setTargetAngleDegBestPath(targetDeg, useMotionMagic);
  }

  @Override
  public void execute() {
    turret.setTargetAngleDegBestPath(targetDeg, useMotionMagic);
  }

  @Override
  public boolean isFinished() {
    return turret.atSetpoint(targetDeg);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    System.out.println(
        "[TurretGoToAngle] target=" + targetDeg
            + "deg shows=" + turret.getAngleDeg()
            + "deg done=" + (!interrupted)
            + " time=" + timer.get() + "s");
  }
}
