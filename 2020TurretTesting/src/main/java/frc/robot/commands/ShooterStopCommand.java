package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterStopCommand extends Command {
  private final ShooterSubsystem shooter;

  public ShooterStopCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
