
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * HopperSubsystem
 *
 * Mechanical responsibility (as you described):
 * - Storage only + extension/contraction to hold more balls.
 *
 * This subsystem is intentionally light right now. It provides placeholders for:
 * - extend / retract commands
 * - "isExtended" telemetry
 *
 * TODO: implement the actuator hardware (motor/pneumatic) once finalized.
 */
public class HopperSubsystem extends SubsystemBase {

  private boolean extended = false;

  public HopperSubsystem() {}

  /** Extend the hopper (placeholder). */
  public void extend() {
    extended = true;
  }

  /** Retract the hopper (placeholder). */
  public void retract() {
    extended = false;
  }

  /** @return true if hopper is currently extended (placeholder). */
  public boolean isExtended() {
    return extended;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Hopper/Extended", extended);
  }
}
