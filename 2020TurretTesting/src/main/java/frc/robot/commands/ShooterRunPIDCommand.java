package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRunPIDCommand extends Command {
  private final ShooterSubsystem shooter;

  public ShooterRunPIDCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.runAtTargetVelocityPID();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
