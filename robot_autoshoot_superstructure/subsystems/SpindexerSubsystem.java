
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * SpindexerSubsystem
 *
 * Purpose:
 * - Move balls at the bottom of the hopper so they present consistently to the TransferSubsystem entry.
 *
 * Notes:
 * - This subsystem should NOT be responsible for "exactly one ball into the shooter". That job is TransferSubsystem
 *   using its throat/exit sensor and feed timing.
 * - This file includes placeholders for sensors. You can move sensors to TransferSubsystem if you decide to keep
 *   all ball-detection in the transfer tunnel only.
 */
public class SpindexerSubsystem extends SubsystemBase {

  private final TalonFX motor;
  private final DutyCycleOut duty = new DutyCycleOut(0.0);

  private double commandedDuty = 0.0;

  public SpindexerSubsystem() {
    motor = new TalonFX(Constants.OperatorConstants.Spindexer.MOTOR_ID, Constants.OperatorConstants.Spindexer.CANBUS_NAME);
    // TODO: configure current limits, neutral mode, inversion, etc.
  }

  /** Run spindexer at a duty cycle in [-1, +1]. */
  public void runDuty(double dutyCycle) {
    commandedDuty = dutyCycle;
    motor.setControl(duty.withOutput(dutyCycle));
  }

  /** Stop spindexer motor. */
  public void stop() {
    runDuty(0.0);
  }

  /** Convenience: run at the configured "base circulation" duty. */
  public void runBase() {
    runDuty(Constants.OperatorConstants.Spindexer.BASE_DUTY);
  }

  /** Convenience: run at the configured "shooting supply" duty. */
  public void runSupply() {
    runDuty(Constants.OperatorConstants.Spindexer.SUPPLY_DUTY);
  }

  /** @return last duty commanded to the motor (telemetry/debug). */
  public double getCommandedDuty() {
    return commandedDuty;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Spindexer/DutyCmd", commandedDuty);
  }
}
