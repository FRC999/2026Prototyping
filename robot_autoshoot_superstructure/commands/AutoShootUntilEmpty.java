
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * One-button "shoot until empty" command.
 *
 * This command is intentionally thin:
 * - It asserts driver intent while scheduled.
 * - The volley state machine (and telemetry) lives in {@link AutoShootSupervisorSubsystem}.
 *
 * Requiring the relevant subsystems prevents other commands from fighting the superstructure
 * while a volley is active.
 */
public class AutoShootUntilEmpty extends Command {

  private final AutoShootSupervisorSubsystem autoShoot;

  public AutoShootUntilEmpty(
      AutoShootSupervisorSubsystem autoShoot,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      SpindexerSubsystem spindexer,
      TransferSubsystem transfer
  ) {
    this.autoShoot = autoShoot;
    addRequirements(autoShoot, turret, shooter, spindexer, transfer);
  }

  @Override
  public void initialize() {
    autoShoot.setShootRequested(true);
  }

  @Override
  public void execute() {
    autoShoot.setShootRequested(true);
  }

  @Override
  public void end(boolean interrupted) {
    autoShoot.setShootRequested(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
