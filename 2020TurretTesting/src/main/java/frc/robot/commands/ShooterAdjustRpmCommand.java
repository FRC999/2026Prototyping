package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAdjustRpmCommand extends Command {
  private final ShooterSubsystem shooter;
  private final double delta;

  public ShooterAdjustRpmCommand(ShooterSubsystem shooter, double delta) {
    this.shooter = shooter;
    this.delta = delta;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.adjustTargetRpm(delta);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
