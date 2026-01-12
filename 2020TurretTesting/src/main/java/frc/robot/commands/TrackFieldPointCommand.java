package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TrackFieldPointCommand extends Command {
  private final TurretSubsystem turret;
  private final DriveSubsystem drive;
  private final double tx;
  private final double ty;
  private final boolean useMotionMagic;

  public TrackFieldPointCommand(TurretSubsystem turret, DriveSubsystem drive, double tx, double ty, boolean useMotionMagic) {
    this.turret = turret;
    this.drive = drive;
    this.tx = tx;
    this.ty = ty;
    this.useMotionMagic = useMotionMagic;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double angleDeg = turret.computeTurretAngleToFieldPointDeg(drive.getPose2d(), tx, ty);
    turret.setTargetAngleDegBestPath(angleDeg, useMotionMagic);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
